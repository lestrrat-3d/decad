package export

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// maxGCodeLayers bounds the layers GCodePerimeters slices.
const maxGCodeLayers = 100000

// GCodeSpec describes the print GCodePerimeters writes G-code for. LayerHeight, LineWidth and
// FilamentDiameter are positive lengths, and FeedRate a positive velocity.
type GCodeSpec struct {
	// LayerHeight is the thickness of a layer.
	LayerHeight units.Value
	// LineWidth is the width of the extruded line, the nozzle's.
	LineWidth units.Value
	// FilamentDiameter is the diameter of the filament the extruder feeds, 1.75 mm for the common one.
	FilamentDiameter units.Value
	// FeedRate is the speed along an extruding move.
	FeedRate units.Value
	// InfillSpacing is the distance between the lines of rectilinear infill, a positive length. The zero value
	// writes none, and a layer is its perimeters alone.
	InfillSpacing units.Value
	// SolidLayers is the number of layers at the bottom and at the top of the print that are filled solid, with
	// lines a line width apart, whatever the InfillSpacing. The zero value makes none, and a negative one is
	// decad.ErrDegenerate.
	SolidLayers int
	// PerimeterInset moves each perimeter half a line width into the material, so that the printed line's edge and
	// not its middle lies on the body's outline (agent-docs/gcode-design.md G10). The infill is unchanged.
	PerimeterInset bool
}

// GCodePerimeters writes the toolpath that prints body's outline layer by layer as G-code
// (agent-docs/gcode-design.md): the perimeter of each layer, traced once along the body's section at the
// middle of the layer, a hole traced as well, and, when the spec sets InfillSpacing, rectilinear infill
// inside the perimeters, its lines alternating between the X and Y directions from one layer to the next. The
// first and last SolidLayers layers are filled solid, their lines a line width apart, and PerimeterInset moves
// each perimeter half a line width into the material. It is the first stage of a slicer, with no skin found
// from what lies above or below, or support, and it answers whether a print would stand: each layer's line is where the part's outline is.
//
// The file uses millimetres, absolute positioning and absolute extrusion (G21, G90, M82). The bed is
// the plane z = 0 of the world and the print starts at the body's lowest point, so layer i is
// printed at z = zmin + (i+1)·LayerHeight, and there are floor((zmax − zmin)/LayerHeight) layers, the
// part above the last whole layer left unprinted. Each loop of the section is a travel move to its
// first point at the layer's height, then extruding moves round it and back to the start. The
// extrusion for a move is its length times the line's cross-section, LayerHeight·LineWidth, divided
// by the filament's cross-section. The outline is the cut of the body's tessellation at tol, a reading of
// the mesh as [decad.Body.SectionOutline] says, so a curved wall is printed as chords.
//
// ctx, w and body MUST NOT be nil, each otherwise decad.ErrDegenerate. A length of the spec MUST be
// positive (zero is decad.ErrDegenerate, a negative value decad.ErrNegativeMagnitude, a wrong kind
// decad.ErrUnitKind), as the feed rate (a velocity); a body lower than one layer, or one needing over
// 100000 layers, is decad.ErrDegenerate. Every failure writes nothing.
func GCodePerimeters(ctx context.Context, w io.Writer, body *decad.Body, tol units.Value, spec GCodeSpec) error {
	if ctx == nil {
		return fmt.Errorf("export: G-code: %w: nil context", decad.ErrDegenerate)
	}
	if w == nil {
		return fmt.Errorf("export: G-code: %w: nil writer", decad.ErrDegenerate)
	}
	if body == nil {
		return fmt.Errorf("export: G-code: %w: nil body", decad.ErrDegenerate)
	}
	positive := func(v units.Value, kind units.Kind, unit units.Unit, what string) (float64, error) {
		x, err := sectionrecord.MagnitudeIn(v, kind, unit, what)
		if err != nil {
			return 0, fmt.Errorf("export: G-code: %w", err)
		}
		if x == 0 {
			return 0, fmt.Errorf("export: G-code: %w: %s must be positive", decad.ErrDegenerate, what)
		}
		return x, nil
	}
	layer, err := positive(spec.LayerHeight, units.Length, units.Millimeter, "the layer height")
	if err != nil {
		return err
	}
	line, err := positive(spec.LineWidth, units.Length, units.Millimeter, "the line width")
	if err != nil {
		return err
	}
	filament, err := positive(spec.FilamentDiameter, units.Length, units.Millimeter, "the filament diameter")
	if err != nil {
		return err
	}
	feed, err := positive(spec.FeedRate, units.Velocity, units.MillimeterPerSecond, "the feed rate")
	if err != nil {
		return err
	}
	box, err := body.Bounds()
	if err != nil {
		return err
	}
	layers := int(math.Floor((box.Max.Z - box.Min.Z) / layer))
	if layers < 1 {
		return fmt.Errorf("export: G-code: %w: the body is lower than one layer", decad.ErrDegenerate)
	}
	if layers > maxGCodeLayers {
		return fmt.Errorf("export: G-code: %w: %d layers is more than %d", decad.ErrDegenerate, layers, maxGCodeLayers)
	}
	var spacing float64
	if spec.InfillSpacing != (units.Value{}) {
		if spacing, err = positive(spec.InfillSpacing, units.Length, units.Millimeter, "the infill spacing"); err != nil {
			return err
		}
	}
	if spec.SolidLayers < 0 {
		return fmt.Errorf("export: G-code: %w: the solid layers are negative", decad.ErrDegenerate)
	}
	perMillimetre := layer * line / (math.Pi * filament * filament / 4)
	var out bytes.Buffer
	fmt.Fprintf(&out, "; perimeters of a %d layer print, %g mm layers\nG21\nG90\nM82\nG92 E0\n", layers, layer)
	extruded := 0.0
	for i := range layers {
		if err := ctx.Err(); err != nil {
			return err
		}
		mid := box.Min.Z + (float64(i)+0.5)*layer
		top := box.Min.Z + float64(i+1)*layer
		plane, err := r3.NewFrame(r3.NewVec(0, 0, mid), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		if err != nil {
			return err
		}
		outline, err := body.SectionOutline(ctx, plane, tol)
		if err != nil {
			return err
		}
		fmt.Fprintf(&out, "; layer %d z=%.4f\n", i, top)
		perimeters := outline.Loops
		if spec.PerimeterInset {
			perimeters = insetLoops(outline.Loops, line/2)
		}
		for _, loop := range perimeters {
			if len(loop) < 2 {
				continue
			}
			fmt.Fprintf(&out, "G0 X%.4f Y%.4f Z%.4f\n", loop[0].X, loop[0].Y, top)
			for k := 1; k <= len(loop); k++ {
				from, to := loop[k-1], loop[k%len(loop)]
				extruded += to.Sub(from).Len() * perMillimetre
				fmt.Fprintf(&out, "G1 X%.4f Y%.4f E%.5f F%.1f\n", to.X, to.Y, extruded, feed*60)
			}
		}
		gap := spacing
		if i < spec.SolidLayers || i >= layers-spec.SolidLayers {
			gap = line // a solid layer, a line against the next
		}
		if gap == 0 {
			continue
		}
		// Layers alternate the infill's direction, so each lies across the one below it.
		for _, seg := range rectilinearInfill(outline.Loops, i%2 == 0, gap, line) {
			fmt.Fprintf(&out, "G0 X%.4f Y%.4f Z%.4f\n", seg.a.X, seg.a.Y, top)
			extruded += seg.b.Sub(seg.a).Len() * perMillimetre
			fmt.Fprintf(&out, "G1 X%.4f Y%.4f E%.5f F%.1f\n", seg.b.X, seg.b.Y, extruded, feed*60)
		}
	}
	if n, err := w.Write(out.Bytes()); err != nil {
		return fmt.Errorf("export: G-code write failed: %w", err)
	} else if n != out.Len() {
		return fmt.Errorf("export: G-code write failed: %w", io.ErrShortWrite)
	}
	return nil
}
