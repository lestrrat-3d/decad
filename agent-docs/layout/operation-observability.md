# decad file layout: operation observability

Operation history and diagnostics internals.
The rules for rows live in `agent-docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/gaptol/` | The gap tolerance a root operation puts on its context, and the default. See gap-tolerance §1. |
| `internal/chaintol/` | The chain tolerance carried by Boolean evaluation contexts. See operation-diagnostics-design. |
| `internal/opdiagnostic/` | Bounded plain-data summaries and aggregation for accepted Boolean and Stitch adjustments. See operation-diagnostics-design. |
| `internal/boolchain/` | Flattens a refused two-body Boolean over mesh Boolean results into source-body literals and runs the regrouped plan. See boolean-chain-regroup-design. |
