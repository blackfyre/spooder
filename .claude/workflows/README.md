# Workflows

## deliver-changes

`deliver-changes.js` takes briefs, existing OpenSpec change names or approvals (`args: {items: [...], push?: true}`) and runs each change, unattended and in parallel, in its own delivery worktree (`mise run worktree-add`):
- propose the change, and have Codex review the proposal;
- apply it task group by task group, with a Codex review of each group;
- run the touched areas' gates, a final Codex review against the spec, and a security review where input arrives from outside;
- update the docs and archive the change (only with no task open; hardware tasks keep it unarchived);
- merge, one change at a time: rebase, re-run the gates, fast-forward `main` and push. A high-risk change (`tools/risk-tier.sh`) stops before the merge until its exact commit is approved with a `{merge: "<name>", approve: "<commit>"}` item.

Every review follows `REVIEW.md`. A blocked or waiting change keeps its worktree for inspection. Merges need `main` checked out in the primary checkout, and refuse to touch its uncommitted work. Each change costs about six agents plus one per task group; only one works on a change at a time.
