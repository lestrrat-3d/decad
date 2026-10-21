# decad agent documentation

This directory holds design decisions and maintenance instructions for agents that develop, fix or maintain decad.
User-facing documentation, including the API reference, lives in [`docs/`](../agent-docs/README.md).

## What belongs here

- `api-design.md`: the design contract and rationale behind the public API. It is not the user reference.
- Design documents (`<topic>-design.md`): the decisions, derivations and refusals behind one area of the code.
- `layout.md` and `layout/`: one row per root `.go` file and per design document.
- `testing.md`: test scope, CI shards and slow-test rules.
- `modules.md`: the record of every `go.mod` module.
- `missing-features.md`: what the library does not do yet and which refusals guard it.

## What does not belong here

- API reference for library users → `docs/api.md` and `docs/data-structures.md`.
- Usage guidance → executable examples in `examples/`.
- Images used by the README → `docs/images/`.

`CLAUDE.md` owns the rules for when to read which file.
