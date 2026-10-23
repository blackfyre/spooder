export const meta = {
  name: 'deliver-changes',
  description: 'Deliver Spooder changes end to end: OpenSpec propose, apply, archive, Codex reviews, merge to main',
  whenToUse: 'Deliver one or more Spooder changes, from a brief or an existing OpenSpec change, through to a merge on main. args: {items: [brief string | {brief, name?} | {change}], push?: true}',
  phases: [
    { title: 'Prepare', detail: 'a delivery worktree per change (mise run worktree-add)' },
    { title: 'Propose', detail: 'write the OpenSpec change from a brief, or read an existing one' },
    { title: 'Review proposal', detail: 'Codex reviews the change artefacts; valid findings applied' },
    { title: 'Apply', detail: 'one agent per task group: implement, gate, Codex review, fix, commit' },
    { title: 'Verify', detail: 'full gates for the touched areas, final Codex review against the specs' },
    { title: 'Archive', detail: 'changelog and docs, then archive the change when no task is open' },
    { title: 'Merge', detail: 'one at a time: rebase, re-run gates, fast-forward main, push' },
  ],
}

// --- Inputs -----------------------------------------------------------------

const ROOT = '/home/meki/Projects/blackfyre/spooder'
const input = args || {}
const PUSH = input.push !== false
const items = (input.items || []).map(it => (typeof it === 'string' ? { brief: it } : it))
if (!items.length) {
  return {
    usage: 'args: {items: ["<brief>" | {brief: "<brief>", name?: "<change-name>"} | {change: "<existing OpenSpec change>"}], push?: true}',
    note: 'Each item is delivered in .claude/worktrees/<name> on deliver/<name>; merges to main run one at a time.',
  }
}
for (const it of items) {
  if (!it.brief && !it.change) throw new Error('every item needs a brief or a change: ' + JSON.stringify(it))
}

// Areas of Spooder and the gates each needs (CLAUDE.md's verification gates).
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

// --- Shared prompt parts ------------------------------------------------------

const AUTHORITY = (name, wt) => `You are one stage of the deliver-changes workflow for Spooder, run unattended at the user's request.
Work only in the worktree ${wt} (branch deliver/${name}); never edit the primary checkout ${ROOT}.
This workflow is the user's explicit authorisation to commit on deliver/${name}. Commit atomically with concise messages; end each with the attribution lines your context gives for commits, if any.
There is no human to ask. Decide what the specifications and the code let you infer, record assumptions in the change's design.md, and report "blocked" with a reason only when a decision cannot be inferred, would need hardware, credentials, destructive operations or a scope well beyond the change, or when the same gate still fails after two substantially different corrections.`

const CODEX = `How to ask Codex (codex CLI) for a review: write the material to a file under your scratchpad directory (or a directory from mktemp -d), then run
  { printf '%s\\n\\n' "<review instructions>"; cat <material file>; } | codex exec -s read-only --ephemeral -C <worktree> -o <scratch>/codex-review.md -
with the Bash tool's timeout at 600000 ms, and read codex-review.md. Codex can read the worktree itself. For each finding decide whether it is valid: fix valid ones (then re-run the affected gates); keep rejected ones with a one-line reason. Do not run more than two Codex rounds for one review.`

const STATUS = { type: 'string', enum: ['ok', 'blocked'] }
const REJECTED = { type: 'array', items: { type: 'object', properties: { finding: { type: 'string' }, reason: { type: 'string' } }, required: ['finding', 'reason'] } }

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
  properties: { status: STATUS, reason: { type: 'string' }, applied: { type: 'array', items: { type: 'string' } }, rejected: REJECTED },
  required: ['status', 'applied', 'rejected'],
}
const GROUP_SCHEMA = {
  type: 'object',
  properties: {
    status: STATUS, reason: { type: 'string' }, commit: { type: 'string' },
    gates: { type: 'array', items: { type: 'object', properties: { command: { type: 'string' }, passed: { type: 'boolean' } }, required: ['command', 'passed'] } },
    hardwareTasks: { type: 'array', items: { type: 'string' } },
    applied: { type: 'array', items: { type: 'string' } },
    rejected: REJECTED,
  },
  required: ['status', 'gates', 'hardwareTasks', 'rejected'],
}
const VERIFY_SCHEMA = {
  type: 'object',
  properties: {
    status: STATUS, reason: { type: 'string' },
    gates: { type: 'array', items: { type: 'object', properties: { command: { type: 'string' }, passed: { type: 'boolean' } }, required: ['command', 'passed'] } },
    openTasks: { type: 'array', items: { type: 'string' } },
    applied: { type: 'array', items: { type: 'string' } },
    rejected: REJECTED,
  },
  required: ['status', 'gates', 'openTasks', 'rejected'],
}
const ARCHIVE_SCHEMA = {
  type: 'object',
  properties: { status: STATUS, reason: { type: 'string' }, archived: { type: 'boolean' }, archivePath: { type: 'string' } },
  required: ['status', 'archived'],
}
const MERGE_SCHEMA = {
  type: 'object',
  properties: { status: STATUS, reason: { type: 'string' }, mainCommit: { type: 'string' }, pushed: { type: 'boolean' } },
  required: ['status', 'pushed'],
}

// Merges run one at a time: each waits for the previous to settle.
let mergeTail = Promise.resolve()
const serialMerge = fn => {
  const run = mergeTail.then(fn)
  mergeTail = run.catch(() => null)
  return run
}

const blocked = ctx => ctx && ctx.status === 'blocked'

// --- Stages -------------------------------------------------------------------

const prepare = (item, _orig, i) => agent(
  `Prepare a delivery worktree for one Spooder change (deliver-changes workflow, item ${i + 1}).
${item.change ? `It delivers the existing OpenSpec change "${item.change}": use that as the name, and check that openspec/changes/${item.change}/ exists on origin/main (git -C ${ROOT} fetch -q origin main; git -C ${ROOT} ls-tree origin/main openspec/changes/${item.change}/).`
    : `It delivers this brief:\n---\n${item.brief}\n---\nChoose the OpenSpec change name${item.name ? ` (use "${item.name}")` : ': kebab-case, verb first, like add-centre-of-gravity'}, and make sure openspec/changes/<name> does not exist yet.`}
Then run, from ${ROOT}: mise run worktree-add <name>
It creates .claude/worktrees/<name> on branch deliver/<name> from origin/main with the submodules ready. Do not change any file.
Return the name, the worktree's absolute path and the base commit (git -C <worktree> rev-parse HEAD). Report blocked if the name exists or the command fails.`,
  { label: `prepare:${item.change || item.name || i + 1}`, phase: 'Prepare', schema: PREPARE_SCHEMA, effort: 'low' },
).then(r => ({ ...r, item }))

const plan = ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, item } = ctx
  const task = item.change
    ? `The change already exists: openspec/changes/${item.change}/. Read its proposal, design, specs and tasks. Do not rewrite it.`
    : `Write the OpenSpec change "${name}" from this brief, with the openspec-propose skill (or the openspec CLI: openspec new change, then the proposal, design, spec deltas and tasks):
---
${item.brief}
---
Follow the repository's existing changes for form: tasks grouped as "## N. Title", each task ending with "Verify ..." naming its deterministic check; spec deltas with scenarios; no task the brief does not need. Run openspec validate ${name} until it passes, and commit the change as "Propose ${name}".`
  return agent(
    `${AUTHORITY(name, worktree)}

${task}

Then report: the change name; the areas of Spooder it touches, from ${AREAS.join(', ')} (protocol: anything in core/protocol or the link frames; core: core/ outside protocol); every task group that still has an unchecked task, with hardwareOnly true when every open task in it needs the real robot or bridge; and the assumptions you made.`,
    { label: `propose:${name}`, phase: 'Propose', schema: PLAN_SCHEMA },
  ).then(r => ({ ...ctx, ...r }))
}

const reviewProposal = ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, change } = ctx
  return agent(
    `${AUTHORITY(name, worktree)}

Have Codex review the OpenSpec change "${change}" (openspec/changes/${change}/) before implementation.
${CODEX}
The material: every file of the change, with the main specs it modifies (openspec/specs/<capability>/spec.md). Ask Codex for: requirements that contradict the main specs or other active changes, behaviour without a scenario, scenarios no task would test, tasks without a deterministic check, work the change does not need, and risks to the 20 ms control tick, power protection or the link protocol.
Apply the valid findings with the openspec-update-change skill (or by editing the artefacts consistently), run openspec validate ${change}, and commit as "Revise ${change} after review". If nothing is valid, change nothing.`,
    { label: `review-proposal:${name}`, phase: 'Review proposal', schema: REVIEW_SCHEMA },
  ).then(r => ({ ...ctx, proposalReview: r, status: r.status, reason: r.reason }))
}

const apply = async ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, change, areas } = ctx
  const groupResults = []
  for (const g of ctx.groups) {
    if (g.hardwareOnly) {
      groupResults.push({ group: g, status: 'ok', skipped: 'hardware only' })
      continue
    }
    const r = await agent(
      `${AUTHORITY(name, worktree)}

Implement task group ${g.number} ("${g.title}") of the OpenSpec change "${change}", with the openspec-apply-change skill's approach: read the change's artefacts and the code they touch, implement the group's unchecked tasks, add or update the tests they call for, and run each task's own Verify check. Leave tasks that need the real robot or bridge unchecked and list them. The change touches: ${areas.join(', ')}.
Before you start, note the commit (git rev-parse HEAD) as this group's baseline.
When the group's checks pass, have Codex review the group:
${CODEX}
The material: git diff <baseline> (including untracked files: git add -N first) and the change's tasks for group ${g.number}. Ask Codex for correctness bugs, behaviour that departs from the spec deltas, missing tests for the group's scenarios, and breaks of the 20 ms tick, protection or protocol rules.
After fixing valid findings and re-running the checks, tick the group's done tasks in tasks.md and commit as "${change}: ${g.title}". Report the commit, the checks with their results, the hardware tasks left open, and the findings applied and rejected.`,
      { label: `apply:${name}#${g.number}`, phase: 'Apply', schema: GROUP_SCHEMA },
    )
    groupResults.push({ group: g, ...r })
    if (!r || r.status === 'blocked') {
      return { ...ctx, groupResults, status: 'blocked', reason: `group ${g.number}: ${r ? r.reason : 'agent failed'}` }
    }
  }
  return { ...ctx, groupResults }
}

const verify = ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, change, areas, base } = ctx
  const gates = gatesFor(areas)
  return agent(
    `${AUTHORITY(name, worktree)}

Verify the whole OpenSpec change "${change}" on deliver/${name}.
1. Run these gates in the worktree, each to completion: ${gates.length ? gates.join('; ') : '(none: docs only; check every path and symbol the changed docs mention exists)'}.
2. Have Codex review the whole change against its specification:
${CODEX}
The material: git diff ${base}..HEAD and the change's spec deltas. Ask Codex whether each requirement and scenario is implemented and tested (list any scenario without a test), for correctness bugs across task groups, and for leftovers (debug output, dead code, unrelated edits).
3. Fix valid findings, re-run the affected gates, commit as "${change}: address review", and finish with every gate passing.
Report each gate's result, the tasks still unchecked in tasks.md (verbatim), and the findings applied and rejected. Report blocked if a gate still fails.`,
    { label: `verify:${name}`, phase: 'Verify', schema: VERIFY_SCHEMA },
  ).then(r => ({ ...ctx, verifyResult: r, status: r.status, reason: r.reason, gates }))
}

const archive = ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, change } = ctx
  const open = (ctx.verifyResult && ctx.verifyResult.openTasks) || []
  return agent(
    `${AUTHORITY(name, worktree)}

Finish the documentation of the OpenSpec change "${change}" and archive it if it is complete.
1. Record the change in CHANGELOG.md under Unreleased (Keep a Changelog sections, in the file's style), and update CLAUDE.md and the project skills in .claude/skills/ where the change makes them wrong or incomplete. Check that every path and symbol you mention exists.
2. ${open.length
      ? `Do not archive: these tasks are still open (${open.join(' | ')}). The change stays in openspec/changes/ for a later delivery.`
      : `Archive it with the openspec-archive-change skill (or openspec archive ${change} -y), syncing its spec deltas into openspec/specs/, and check openspec validate --specs passes.`}
3. Commit as "Archive ${change}" (or "Document ${change}" when not archived).
Report whether it was archived and where to.`,
    { label: `archive:${name}`, phase: 'Archive', schema: ARCHIVE_SCHEMA },
  ).then(r => ({ ...ctx, archiveResult: r, status: r.status, reason: r.reason }))
}

const merge = ctx => {
  if (blocked(ctx)) return ctx
  const { name, worktree, change, gates } = ctx
  return serialMerge(() => agent(
    `${AUTHORITY(name, worktree)}
This stage is also the user's explicit authorisation to fast-forward main in ${ROOT}${PUSH ? ' and push main to origin' : ''}. Never force-push, never rewrite main, never touch uncommitted work in ${ROOT}.

Merge deliver/${name} (the OpenSpec change "${change}") into main:
1. git -C ${ROOT} fetch origin main. ${PUSH ? `Local main must equal origin/main (git -C ${ROOT} rev-parse main origin/main); if not, report blocked.` : ''}
2. In the worktree, rebase onto main (git rebase main). Resolve conflicts that are mechanical (changelog entries, adjacent edits) keeping both sides; report blocked on a conflict of behaviour.
3. Re-run the gates on the rebased branch: ${gates.length ? gates.join('; ') : '(none)'}. All must pass.
4. Check that ${ROOT} has main checked out (git -C ${ROOT} branch --show-current), then git -C ${ROOT} merge --ff-only deliver/${name}. If git refuses because of uncommitted changes there, report blocked: leave them alone.
5. ${PUSH ? `git -C ${ROOT} push origin main (no force). If the push is rejected, report blocked.` : 'Do not push.'}
6. From ${ROOT}: mise run worktree-remove ${name}.
Report main's new commit and whether it was pushed.`,
    { label: `merge:${name}`, phase: 'Merge', schema: MERGE_SCHEMA },
  )).then(r => ({ ...ctx, mergeResult: r, status: r.status, reason: r.reason }))
}

// --- Run ----------------------------------------------------------------------

log(`Delivering ${items.length} change(s); merges one at a time${PUSH ? ', each pushed' : ', not pushed'}.`)
const results = await pipeline(items, prepare, plan, reviewProposal, apply, verify, archive, merge)

const report = results.map((r, i) => {
  if (!r) return { item: items[i], status: 'failed', reason: 'a stage threw or an agent died' }
  return {
    change: r.change || r.name,
    status: r.status,
    reason: r.reason,
    worktree: r.status === 'blocked' ? r.worktree : undefined,
    assumptions: r.assumptions,
    hardwareTasks: [].concat(...(r.groupResults || []).map(g => g.hardwareTasks || (g.skipped ? [`group ${g.group.number}: ${g.group.title}`] : []))),
    openTasks: r.verifyResult && r.verifyResult.openTasks,
    archived: r.archiveResult && r.archiveResult.archived,
    mainCommit: r.mergeResult && r.mergeResult.mainCommit,
    pushed: r.mergeResult && r.mergeResult.pushed,
    rejectedFindings: [].concat(
      (r.proposalReview && r.proposalReview.rejected) || [],
      ...(r.groupResults || []).map(g => g.rejected || []),
      (r.verifyResult && r.verifyResult.rejected) || [],
    ),
  }
})
for (const r of report) log(`${r.change || 'item'}: ${r.status}${r.reason ? ' (' + r.reason + ')' : ''}`)
return report
