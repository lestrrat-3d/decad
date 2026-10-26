# Text design

`TextProfiles` draws a line of lettering as sketch profiles, so `Emboss` can raise or engrave it. It lays the text out
in a TrueType font, flattens each glyph outline to straight lines and draws them into a new sketch. The layout is
plain-data code in `internal/glyphs/`, over the `golang.org/x/image/font/sfnt` reader (`agent-docs/modules.md`).

## 1. Contract

```go
type TextSpec struct {
    Text      string
    Height    units.Value
    At        Point2
    Tolerance units.Value
    Font      []byte
}

func TextProfiles(ctx context.Context, w *sketch.World, plane *sketch.Plane, spec TextSpec) (*sketch.Sketch, []*sketch.Profile, error)
```

| Decision | Rule |
|---|---|
| T1 | The text is set in the font given, or in Go Regular when `Font` is nil. `Font` is TrueType data that `sfnt.Parse` reads, otherwise `ErrDegenerate`. |
| T2 | `Height` is the length of one em, a positive length. A zero is `ErrDegenerate`, a negative value `ErrNegativeMagnitude`, and a wrong unit kind `ErrUnitKind`. A capital of Go Regular is about 0.7 of it tall. |
| T3 | The first line's baseline starts at `At`, in the sketch plane's coordinates, and the y axis runs up. A rune is set at the pen and the pen then moves by the glyph's advance width. There is no kerning and no shaping. |
| T4 | `"\n"` ends a line. Each later line is one line height of the font below the one before, its pen back at the start. |
| T5 | A rune the font has no glyph for, text that is empty or only white space, and text that draws no outline are `ErrDegenerate`. A space moves the pen and draws nothing. |
| T6 | A curve is flattened until it lies within `Tolerance` of its polygon: a positive length, or a thousandth of `Height` when the zero `units.Value`. Text that would need more than 200000 outline points is `ErrDegenerate`. |
| T7 | Each contour is a closed ring of fixed sketch points and `Line`s, drawn into a new sketch on `plane`. `ctx` and `w` and `plane` MUST NOT be nil. |
| T8 | The returned profiles are the regions the font fills. A region inside another's hole is empty, and one inside that empty region's hole is filled again: a region is filled when an even number of regions enclose it, counted by the loop that bounds it. So an `o` is one profile with a hole, and its counter is not returned. |
| T9 | A region that is not a valid profile, which is where letters touch or overlap, is `ErrDegenerate`. |

## 2. Using it

The sketch is on `plane`, so draw the lettering on the plane of the face it marks, which `Emboss` requires of its
sketch (`agent-docs/emboss-design.md` E2). A raised label is `Emboss(.., EmbossRaised, height)` and an engraving
`EmbossEngraved`. A round letter is a polygon of straight lines; a smaller `Tolerance` makes it rounder at the cost of
more lines in the sketch.

## 3. Verification

The tests read Go Regular's own metrics with `sfnt` and compare: a capital I's polygon bounds and area against the
font's glyph box and the shoelace area of its straight segments; an O as a ring with one hole whose counter is not
returned; the second of two letters one advance from the first; lines one line height apart; and a plate whose volume
rises or falls by the letters' area times the relief height. A filter that keeps every region or a y axis the wrong way
round turns them red.

## 4. Not covered

- Kerning, ligatures, right-to-left text, bold and italic synthesis, and any layout beyond lines of left-to-right runes.
- A font that is not TrueType outlines, such as a bitmap or colour font.
- Text along a curve, text in a box that wraps, and text that follows a face's surface.
- True curves: the letters are polygons, so a letter's faces are planes.
