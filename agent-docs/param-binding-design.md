# Parameter binding

Companion to `agent-docs/api-design.md` §6 and §13.

## 1. What it is

A feature value can be derived from named parameters. The document reads them from one `sketch/param` table, so a
sketch dimension and a feature value share definitions. A translator that reads the model's Go source (a Fusion
add-in generator) sees the parameter names and expressions as literals and can check its reading against a run.

The model stays immediate-mode: the caller's Go function is the feature tree (api-design §6). There is no recorded
feature history and no replay. An edit is `Table.Set` followed by running the builder again.

## 2. API

| Symbol | Role |
|---|---|
| `WithParams(*param.Table)` | `New` option. The document reads this table, for example `sketch.World.Params()`. Without it the document owns a fresh table. |
| `Document.Params()` | The table. Definitions are made on it with `Set`, `SetExpr` and `SetValue`. |
| `Document.Param(name)` | The parameter as a `units.Value` in its declared unit. |
| `Document.Expr(expr)` | An ad-hoc expression as a `units.Value` in the base unit of the kind it computes to. |
| `Document.ParamInt(name)` | A whole dimensionless parameter, for a count. Anything else is `ErrDegenerate`. |
| `Document.Bindings()` | Every successful `Param` and `Expr` read, in call order. |

No feature signature changes: a feature takes the `units.Value` that `Param` returns, so a plain number and a
parameter use the same call form.

## 3. Ledger

A `Binding` holds the parameter name (empty for `Expr`), its source text, the value read and the parameters its
expression references directly. It records reads of the table, not features: a read happens before the feature call
that uses it, and the ledger does not link the two. A failed read records nothing. Dependencies of an ad-hoc
expression are not listed because `param` exposes no reference walk for one.

## 4. Rules

- The kind check is `param`'s: a length minus a bare number is `param.ErrIncompatibleKind`, so a unit mistake
  fails at the read instead of reaching a feature.
- The document never copies the table. A later `Set` changes what the next read returns.
- `internal/parambind` holds the ledger; the root keeps the exported methods (CLAUDE.md, Conventions).
- Importing `sketch/param` adds no module: it is a package of the `sketch` module decad already requires.
- Parameter definitions the translator must read are written as `Set("name", "expr")` calls with string literals.
