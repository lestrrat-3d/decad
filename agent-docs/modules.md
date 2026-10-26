# Approved modules

Adding a `go.mod` module requires recording the decision here first.

- `github.com/lestrrat-3d/sketch` — parametric 2D constraint engine.
- `github.com/lestrrat-3d/r3` — 3D coordinate math (`Vec`, `Frame`, `Transform`).
- `github.com/lestrrat-3d/units` — typed quantities (`Value`, `Kind`).
  decad's inputs and `Measurement`s are `units.Value`, the module `sketch`
  uses too, so no parallel unit system exists.
- `github.com/lestrrat-3d/step` — AP214; `export` package only.
- `golang.org/x/image` — the `font/sfnt` TrueType reader and the `font/gofont/goregular`
  font, for `TextProfiles` alone (`internal/glyphs/`). It pulls in `golang.org/x/text`.
  decad draws lettering from glyph outlines it flattens itself, so no rendering code
  of the module is used, and the font is the Go project's own BSD-licensed Go Regular.
- `github.com/lestrrat-go/option/v3` — functional options (house library). Used
  by feature options.
- `github.com/stretchr/testify/require` — assertions, **test code only**.
  NEVER import from production code.
- `_gallery` module ALONE; the root `go.mod`/`go.sum` never list them:
  - `github.com/lestrrat-3d/solidlens` — pure-Go mesh rasterizer; README
    images.
  - `github.com/lestrrat-3d/kinetograph` — animates decad bodies; landing
    clip (`go run . clip`) and dynamics scenes (`go run . dynamics`).
