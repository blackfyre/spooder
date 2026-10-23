export const meta = {
  name: 'deliver-changes',
  description: 'Deliver Spooder changes end to end: OpenSpec propose, apply, archive, Codex reviews, risk-tiered merge to main',
  whenToUse: 'Deliver one or more Spooder changes, from a brief or an existing OpenSpec change, through to a merge on main; or merge an approved high-risk change. args: {items: [brief string | {brief, name?} | {change} | {merge, approve}], push?: true}',
  phases: [
    { title: 'Prepare', detail: 'a delivery worktree per change (mise run worktree-add)' },
    { title: 'Propose', detail: 'write the OpenSpec change from a brief, or read an existing one' },
    { title: 'Review proposal', detail: 'Codex reviews the change artefacts; valid findings applied' },
    { title: 'Apply', detail: 'one agent per task group: implement, gate, Codex review, fix, commit' },
    { title: 'Verify', detail: 'gates, final Codex review against the specs, security review where input arrives' },
    { title: 'Archive', detail: 'changelog and docs, then archive the change when no task is open' },
    { title: 'Merge', detail: 'one at a time: rebase, re-run gates, risk tier, fast-forward main, push' },
  ],
}

// --- Inputs -----------------------------------------------------------------

const ROOT = '/home/meki/Projects/blackfyre/spooder'
const input = args || {}
const PUSH = input.push !== false
const items = (input.items || []).map(it => (typeof it === 'string' ? { brief: it } : it))
if (!items.length) {
  return {
    usage: 'args: {items: ["<brief>" | {brief: "<brief>", name?: "<change-name>"} | {change: "<existing OpenSpec change>"} | {merge: "<name>", approve: "<commit>"}], push?: true}',
    note: 'Each change is delivered in .claude/worktrees/<name> on deliver/<name>; merges to main run one at a time. A high-risk change (tools/risk-tier.sh) stops before its merge until you approve its commit with a {merge, approve} item.',
  }
}
for (const it of items) {
  if (!it.brief && !it.change && !it.merge) throw new Error('every item needs a brief, a change or a merge: ' + JSON.stringify(it))
  if (it.merge && !it.approve) throw new Error('a merge item needs the approved commit: ' + JSON.stringify(it))
}

// Areas of Spooder and the gates each needs (the root CLAUDE.md's verification gates).
const AREAS = ['core', 'firmware', 'bridge', 'protocol', 'desktop', 'virtual-robot', 'tooling', 'docs']
const GATES = {
  core: ['mise run test'],
  firmware: ['mise run build', 'mise run build --serial'],
  bridge: ['mise run build-bridge'],
  protocol: ['mise run test', 'mise run build', 'mise run build --serial', 'mise run build-bridge',
    'mise run desktop-test', 'mise run desktop-build', 'mise run e2e'],
  desktop: ['mise run desktop-test', 'mise run desktop-build', 'mise run e2e'],
  'virtual-robot': ['mise run virtual-robot', 'mise run e2e'],
  tooling: ['mise run test'],
  docs: [],
}
const gatesFor = areas => {
  const set = new Set()
  for (const a of areas) for (const g of GATES[a] || []) set.add(g)
  // core/ is shared by both firmwares and the virtual robot.
  if (areas.includes('core')) ['mise run build', 'mise run build --serial', 'mise run build-bridge', 'mise run e2e'].forEach(g => set.add(g))
  return [...set]
}
// Areas where input arrives from outside (REVIEW.md, Security).
const SECURITY_AREAS = ['protocol', 'bridge', 'desktop', 'tooling']

// --- Shared prompt parts ------------------------------------------------------

const AUTHORITY = (name, wt) => `You are one stage of the deliver-changes workflow for Spooder, run unattended at the user's request.
Work only in the worktree ${wt} (branch deliver/${name}); never edit the primary checkout ${ROOT}.
This workflow is the user's explicit authorisation to commit on deliver/${name}. Commit atomically with concise messages; end each with the attribution lines your context gives for commits, if any.
There is no human to ask. Decide what the specifications and the code let you infer, record assumptions in the change's design.md, and report "blocked" with a reason only when a decision cannot be inferred, would need hardware, credentials, destructive operations or a scope well beyond the change, or when the same gate still fails after two substantially different corrections.`

// How every review runs (REVIEW.md, and the convergence rules: a round cap,
// diff-only re-review, no repeats, bug classes, pre-existing kept apart).
const CODEX = `How to review with Codex (codex CLI), following REVIEW.md at the worktree root (what failure looks like, severity tags, evidence, the nit cap, paths to skip, the summary format):
- Round 1: write the material to a file under your scratchpad directory (or a directory from mktemp -d) and run
    { printf '%s\\n\\n' "<review instructions, ending: Follow REVIEW.md.>"; cat <material file>; } | codex exec -s read-only --ephemeral -C <worktree> -o <scratch>/codex-review-1.md -
  with the Bash tool's timeout at 600000 ms, then read the output. Codex can read the worktree itself.
- Answer every finding: fix it (outcome fixed), or close it with a reason as wrong, intentional or didnt-matter. A pre-existing finding (present before this change: check with git blame or the base commit) is not fixed here: outcome pre-existing, reported for later. When a finding names a class of bug, fix every instance.
- Later rounds, only while round 1's fixes leave Important findings to check: give Codex only the diff since the previous round and the findings already answered, and tell it not to repeat them. At most three rounds. If Important findings remain after round three, stop and report blocked with "reconsider the approach" and the findings.`

const STATUS = { type: 'string', enum: ['ok', 'blocked'] }
const FINDINGS = {
  type: 'array',
  items: {
    type: 'object',
    properties: {
      finding: { type: 'string' },
      where: { type: 'string' },
      severity: { type: 'string', enum: ['important', 'nit', 'pre-existing'] },
      outcome: { type: 'string', enum: ['fixed', 'wrong', 'intentional', 'didnt-matter', 'pre-existing'] },
      reason: { type: 'string' },
    },
    required: ['finding', 'severity', 'outcome'],
  },
}
const GATE_RESULTS = { type: 'array', items: { type: 'object', properties: { command: { type: 'string' }, passed: { type: 'boolean' } }, required: ['command', 'passed'] } }

const PREPARE_SCHEMA = {
  type: 'object',
  properties: { status: STATUS, reason: { type: 'string' }, name: { type: 'string' }, worktree: { type: 'string' }, base: { type: 'string' } },
  required: ['status', 'name', 'worktree'],
}
const PLAN_SCHEMA = {
  type: 'object',
  properties: {
    status: STATUS, reason: { type: 'string' },
    change: { type: 'string' },
    areas: { type: 'array', items: { type: 'string', enum: AREAS } },
    groups: {
      type: 'array',
      items: {
        type: 'object',
        properties: { number: { type: 'integer' }, title: { type: 'string' }, hardwareOnly: { type: 'boolean' } },
        required: ['number', 'title', 'hardwareOnly'],
      },
    },
    assumptions: { type: 'array', items: { type: 'string' } },
  },
  required: ['status', 'change', 'areas', 'groups'],
}
const REVIEW_SCHEMA = {
  type: 'object',
  properties: { status: STATUS, reason: { type: 'string' }, rounds: { type: 'integer' }, findings: FINDINGS },
  required: ['status', 'rounds', 'findings'],
}
const GROUP_SCHEMA = {
  type: 'object',
  properties: {
    status: STATUS, reason: { type: 'string' }, commit: { type: 'string' },
    gates: GATE_RESULTS,
    hardwareTasks: { type: 'array', items: { type: 'string' } },
    rounds: { type: 'integer' },
    findings: FINDINGS,
  },
  required: ['status', 'gates', 'hardwareTasks', 'rounds', 'findings'],
}
const VERIFY_SCHEMA = {
  type: 'object',
  properties: {
    status: STATUS, reason: { type: 'string' },
    gates: GATE_RESULTS,
    tier: { type: 'string', enum: ['low', 'medium', 'high'] },
    openTasks: { type: 'array', items: { type: 'string' } },
    rounds: { type: 'integer' },
    findings: FINDINGS,
    securityReviewed: { type: 'boolean' },
    securityFindings: FINDINGS,
  },
  required: ['status', 'gates', 'tier', 'openTasks', 'rounds', 'findings', 'securityReviewed'],
}
const ARCHIVE_SCHEMA = {
  type: 'object',
  properties: { status: STATUS, reason: { type: 'string' }, archived: { type: 'boolean' }, archivePath: { type: 'string' } },
  required: ['status', 'archived'],
}
const MERGE_SCHEMA = {
  type: 'object',
  properties: {
    status: { type: 'string', enum: ['ok', 'blocked', 'awaiting-approval'] },
    reason: { type: 'string' },
    tier: { type: 'string', enum: ['low', 'medium', 'high'] },
    tierPaths: { type: 'array', items: { type: 'string' } },
    branchCommit: { type: 'string' },
    gates: GATE_RESULTS,
    mainCommit: { type: 'string' },
    pushed: { type: 'boolean' },
    revert: { type: 'string' },
  },
  required: ['status', 'pushed'],
}

// Merges run one at a time: each waits for the previous to settle.
let mergeTail = Promise.resolve()
const serialMerge = fn => {
  const run = mergeTail.then(fn)
  mergeTail = run.catch(() => null)
  return run
}

const stop = ctx => !ctx || ctx.status !== 'ok' || ctx.mergeOnly

// --- Stages -------------------------------------------------------------------

const prepare = (item, _orig, i) => {
  if (item.merge) {
    // A high-risk change approved by its commit: only the merge stage runs.
    return Promise.resolve({ status: 'ok', name: item.merge, worktree: `${ROOT}/.claude/worktrees/${item.merge}`, mergeOnly: true, item })
  }
  return agent(
    `Prepare a delivery worktree for one Spooder change (deliver-changes workflow, item ${i + 1}).
${item.change ? `It delivers the existing OpenSpec change "${item.change}": use that as the name, and check that openspec/changes/${item.change}/ exists on origin/main (git -C ${ROOT} fetch -q origin main; git -C ${ROOT} ls-tree origin/main openspec/changes/${item.change}/).`
      : `It delivers this brief:\n---\n${item.brief}\n---\nChoose the OpenSpec change name${item.name ? ` (use "${item.name}")` : ': kebab-case, verb first, like add-centre-of-gravity'}, and make sure openspec/changes/<name> does not exist yet.`}
Then run, from ${ROOT}: mise run worktree-add <name>
It creates .claude/worktrees/<name> on branch deliver/<name> from origin/main with the submodules ready. Do not change any file.
Return the name, the worktree's absolute path and the base commit (git -C <worktree> rev-parse HEAD). Report blocked if the name exists or the command fails.`,
    { label: `prepare:${item.change || item.name || i + 1}`, phase: 'Prepare', schema: PREPARE_SCHEMA, effort: 'low' },
  ).then(r => ({ ...r, item }))
}

const plan = ctx => {
  if (stop(ctx)) return ctx
  const { name, worktree, item } = ctx
  const task = item.change
    ? `The change already exists: openspec/changes/${item.change}/. Read its proposal, design, specs and tasks. Do not rewrite it.`
    : `Write the OpenSpec change "${name}" from this brief, with the openspec-propose skill (or the openspec CLI: openspec new change, then the proposal, design, spec deltas and tasks):
---
${item.brief}
---
Follow the repository's existing changes for form: tasks grouped as "## N. Title", each task ending with "Verify ..." naming its deterministic check; spec deltas with scenarios; no task the brief does not need. Keep the change small: when the brief holds independent parts, deliver the smallest coherent one and record the rest as follow-ups in design.md. Run openspec validate ${name} until it passes, and commit the change as "Propose ${name}".`
  return agent(
    `${AUTHORITY(name, worktree)}

${task}

Then report: the change name; the areas of Spooder it touches, from ${AREAS.join(', ')} (protocol: anything in core/protocol or the link frames; core: core/ outside protocol); every task group that still has an unchecked task, with hardwareOnly true when every open task in it needs the real robot or bridge; and the assumptions you made.`,
    { label: `propose:${name}`, phase: 'Propose', schema: PLAN_SCHEMA },
  ).then(r => ({ ...ctx, ...r }))
}

const reviewProposal = ctx => {
  if (stop(ctx)) return ctx
  const { name, worktree, change } = ctx
  return agent(
    `${AUTHORITY(name, worktree)}

Have Codex review the OpenSpec change "${change}" (openspec/changes/${change}/) before implementation.
${CODEX}
The material: every file of the change, with the main specs it modifies (openspec/specs/<capability>/spec.md). Ask Codex for: requirements that contradict the main specs or other active changes, behaviour without a scenario, scenarios no task would test, tasks without a deterministic check, work the change does not need, and risks to the 20 ms control tick, power protection or the link protocol.
Apply the fixed findings with the openspec-update-change skill (or by editing the artefacts consistently), run openspec validate ${change}, and commit as "Revise ${change} after review". If nothing needs fixing, change nothing.`,
    { label: `review-proposal:${name}`, phase: 'Review proposal', schema: REVIEW_SCHEMA },
  ).then(r => ({ ...ctx, proposalReview: r, status: r.status, reason: r.reason }))
}

const apply = async ctx => {
  if (stop(ctx)) return ctx
  const { name, worktree, change, areas } = ctx
  const groupResults = []
  for (const g of ctx.groups) {
    if (g.hardwareOnly) {
      groupResults.push({ group: g, status: 'ok', skipped: 'hardware only' })
      continue
    }
    const r = await agent(
      `${AUTHORITY(name, worktree)}

Implement task group ${g.number} ("${g.title}") of the OpenSpec change "${change}", with the openspec-apply-change skill's approach: read the change's artefacts and the code they touch, implement the group's unchecked tasks, add or update the tests they call for, and run each task's own Verify check, showing its output in your notes rather than asserting it passed. Leave tasks that need the real robot or bridge unchecked and list them. The change touches: ${areas.join(', ')}.
Before you start, note the commit (git rev-parse HEAD) as this group's baseline.
When the group's checks pass, have Codex review the group:
${CODEX}
The material: git diff <baseline> (including untracked files: git add -N first) and the change's tasks for group ${g.number}. Ask Codex for correctness bugs, behaviour that departs from the spec deltas, missing tests for the group's scenarios, and breaks of the 20 ms tick, protection or protocol rules.
After the fixes, re-run the checks, tick the group's done tasks in tasks.md and commit as "${change}: ${g.title}". Report the commit, the checks with their results, the hardware tasks left open, the review rounds and every finding with its severity and outcome.`,
      { label: `apply:${name}#${g.number}`, phase: 'Apply', schema: GROUP_SCHEMA },
    )
    groupResults.push({ group: g, ...r })
    if (!r || r.status !== 'ok') {
      return { ...ctx, groupResults, status: 'blocked', reason: `group ${g.number}: ${r ? r.reason : 'agent failed'}` }
    }
  }
  return { ...ctx, groupResults }
}

const verify = ctx => {
  if (stop(ctx)) return ctx
  const { name, worktree, change, areas, base } = ctx
  const gates = gatesFor(areas)
  const security = areas.some(a => SECURITY_AREAS.includes(a))
  return agent(
    `${AUTHORITY(name, worktree)}

Verify the whole OpenSpec change "${change}" on deliver/${name}.
1. Run these gates in the worktree, each to completion: ${gates.length ? gates.join('; ') : '(none: docs only; check every path and symbol the changed docs mention exists)'}.
2. Run tools/risk-tier.sh ${base} and report its tier.
3. Have Codex review the whole change against its specification:
${CODEX}
The material: git diff ${base}..HEAD and the change's spec deltas. Ask Codex whether each requirement and scenario is implemented and tested (list any scenario without a test), for correctness bugs across task groups, and for leftovers (debug output, dead code, unrelated edits).
4. ${security ? 'Run a separate security review, as REVIEW.md\'s Security section describes: with the security-review skill if you have it, otherwise a second Codex review with that section as its instructions. Fix its Important findings the same way; no Important security finding may stay open.'
      : `A separate security review is needed only if the tier is high; if it is, run one as REVIEW.md's Security section describes (the security-review skill, or a Codex review with that section as its instructions).`}
5. Fix, re-run the affected gates, commit as "${change}: address review", and finish with every gate passing.
Report each gate's result, the tier, the tasks still unchecked in tasks.md (verbatim), the review rounds, every finding with severity and outcome, and whether a security review ran with its findings. Report blocked if a gate still fails or an Important finding is still open.`,
    { label: `verify:${name}`, phase: 'Verify', schema: VERIFY_SCHEMA },
  ).then(r => ({ ...ctx, verifyResult: r, status: r.status, reason: r.reason, gates }))
}

const archive = ctx => {
  if (stop(ctx)) return ctx
  const { name, worktree, change } = ctx
  const open = (ctx.verifyResult && ctx.verifyResult.openTasks) || []
  return agent(
    `${AUTHORITY(name, worktree)}

Finish the documentation of the OpenSpec change "${change}" and archive it if it is complete.
1. Record the change in CHANGELOG.md under Unreleased (Keep a Changelog sections, in the file's style), and update the CLAUDE.md files (the root one and the area's: core/, firmware/servo2040/, firmware/bridge/, desktop/, tools/, test/) and the project skills in .claude/skills/ where the change makes them wrong or incomplete. When a review in this delivery flagged the same kind of problem more than once, add the rule that would have prevented it to REVIEW.md, the relevant CLAUDE.md or skill. Check that every path and symbol you mention exists.
2. ${open.length
      ? `Do not archive: these tasks are still open (${open.join(' | ')}). The change stays in openspec/changes/ for a later delivery.`
      : `Archive it with the openspec-archive-change skill (or openspec archive ${change} -y), syncing its spec deltas into openspec/specs/, and check openspec validate --specs passes.`}
3. Commit as "Archive ${change}" (or "Document ${change}" when not archived).
Report whether it was archived and where to.`,
    { label: `archive:${name}`, phase: 'Archive', schema: ARCHIVE_SCHEMA },
  ).then(r => ({ ...ctx, archiveResult: r, status: r.status, reason: r.reason }))
}

const ALL_GATES = gatesFor(['protocol', 'virtual-robot'])

const merge = ctx => {
  if (!ctx || ctx.status !== 'ok') return ctx
  const { name, worktree, item } = ctx
  const change = ctx.change || name
  const gates = ctx.mergeOnly ? ALL_GATES : ctx.gates
  const approved = ctx.mergeOnly ? item.approve : null
  return serialMerge(() => agent(
    `${AUTHORITY(name, worktree)}
This stage is also the user's explicit authorisation to fast-forward main in ${ROOT}${PUSH ? ' and push main to origin' : ''}, subject to the risk tier below. Never force-push, never rewrite main, never touch uncommitted work in ${ROOT}.

Merge deliver/${name} (the change "${change}") into main:
1. git -C ${ROOT} fetch origin main. ${PUSH ? `Local main must equal origin/main (git -C ${ROOT} rev-parse main origin/main); if not, report blocked.` : ''}
${approved
      ? `2. The user approved commit ${approved}. The worktree's HEAD must be that commit before you rebase (compare full hashes, a prefix is fine); if not, report awaiting-approval with the current HEAD as branchCommit.
3. Note the approved change's patch id: git diff $(git merge-base ${approved} main) ${approved} | git patch-id --stable. Rebase onto main (git rebase main). The approval covers the result only when the rebase needed no conflict resolved by hand and git diff main HEAD | git patch-id --stable gives the same patch id (the identical change on the newer main). Otherwise resolve what is needed, re-run the gates, and report awaiting-approval with the new HEAD as branchCommit instead of merging.`
      : `2. Rebase onto main (git rebase main). Resolve conflicts that are mechanical (changelog entries, adjacent edits) keeping both sides; report blocked on a conflict of behaviour.
3. Run tools/risk-tier.sh main and report its tier and paths. If the tier is high, do not merge: run the gates (step 4), then stop and report awaiting-approval with HEAD as branchCommit, leaving the worktree in place; the user approves that exact commit with a {merge: "${name}", approve: "<commit>"} item.`}
4. Re-run the gates on the rebased branch: ${gates.length ? gates.join('; ') : '(none)'}. All must pass.
5. Check that ${ROOT} has main checked out (git -C ${ROOT} branch --show-current), note main's commit as <before>, then git -C ${ROOT} merge --ff-only deliver/${name}. If git refuses because of uncommitted changes there, report blocked: leave them alone.
6. ${PUSH ? `git -C ${ROOT} push origin main (no force). If the push is rejected, report blocked.` : 'Do not push.'}
7. From ${ROOT}: mise run worktree-remove ${name}.
Report the tier, each gate's result from step 4, main's new commit, whether it was pushed, and how to revert the change: git revert --no-edit <before>..<new main commit>.`,
    { label: `merge:${name}`, phase: 'Merge', schema: MERGE_SCHEMA },
  )).then(r => ({ ...ctx, mergeResult: r, status: r.status, reason: r.reason }))
}

// --- Run ----------------------------------------------------------------------

log(`Delivering ${items.length} item(s); merges one at a time${PUSH ? ', each pushed' : ', not pushed'}; high-risk changes wait for approval.`)
const results = await pipeline(items, prepare, plan, reviewProposal, apply, verify, archive, merge)

const allFindings = r => [].concat(
  (r.proposalReview && r.proposalReview.findings) || [],
  ...(r.groupResults || []).map(g => g.findings || []),
  (r.verifyResult && r.verifyResult.findings) || [],
  (r.verifyResult && r.verifyResult.securityFindings) || [],
)
const outcomeCounts = fs => fs.reduce((acc, f) => ({ ...acc, [f.outcome]: (acc[f.outcome] || 0) + 1 }), {})
const reviewRounds = r => [
  r.proposalReview && r.proposalReview.rounds,
  ...(r.groupResults || []).map(g => g.rounds),
  r.verifyResult && r.verifyResult.rounds,
].filter(n => typeof n === 'number')

const report = results.map((r, i) => {
  if (!r) return { item: items[i], status: 'failed', reason: 'a stage threw or an agent died' }
  const findings = allFindings(r)
  const m = r.mergeResult || {}
  return {
    change: r.change || r.name,
    status: r.status,
    reason: r.reason,
    tier: m.tier || (r.verifyResult && r.verifyResult.tier),
    awaitingApproval: r.status === 'awaiting-approval' ? { commit: m.branchCommit, approveWith: { merge: r.name, approve: m.branchCommit } } : undefined,
    worktree: r.status !== 'ok' ? r.worktree : undefined,
    testing: m.gates || (r.verifyResult && r.verifyResult.gates), // the final gates, on the rebased branch
    securityReviewed: r.verifyResult && r.verifyResult.securityReviewed,
    assumptions: r.assumptions,
    hardwareTasks: [].concat(...(r.groupResults || []).map(g => g.hardwareTasks || (g.skipped ? [`group ${g.group.number}: ${g.group.title}`] : []))),
    openTasks: r.verifyResult && r.verifyResult.openTasks,
    archived: r.archiveResult && r.archiveResult.archived,
    mainCommit: m.mainCommit,
    pushed: m.pushed,
    revert: m.revert,
    reviewRounds: reviewRounds(r),
    findingOutcomes: outcomeCounts(findings),
    preExisting: findings.filter(f => f.outcome === 'pre-existing'),
    closedWithoutFix: findings.filter(f => ['wrong', 'intentional', 'didnt-matter'].includes(f.outcome)),
  }
})
for (const r of report) log(`${r.change || 'item'}: ${r.status}${r.reason ? ' (' + r.reason + ')' : ''}${r.tier ? ', ' + r.tier + ' risk' : ''}`)
return report
