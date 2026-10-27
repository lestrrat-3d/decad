package export

import (
	"bytes"
	"context"
	"fmt"
	"io"
	"math"
	"strings"

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
	// Retraction is the length of filament pulled back before each travel move between two extruded lines and pushed
	// forward again after it, a positive length. The zero value writes no retraction. The first travel of the print,
	// which follows no extrusion, has none.
	Retraction units.Value
	// NozzleTemperature and BedTemperature, when set, heat the nozzle and the bed to that temperature, waiting for it,
	// before the first layer, and turn both off after the last. A temperature is positive in degrees Celsius and below
	// 1000 degrees, and the zero values leave the heaters alone.
	NozzleTemperature, BedTemperature units.AffineValue
	// StartCode and EndCode are lines written as they are after the header and the heating, and after the last layer:
	// the machine's own start and end sequence. A line break in them starts a line of the file.
	StartCode, EndCode string
	// FanSpeed, a dimensionless fraction above 0 and at most 1, turns the part cooling fan on at that share of its full
	// speed (M106) at the start of layer FanFromLayer, counted from zero, and off (M107) after the last layer. The zero
	// value leaves the fan alone. FanFromLayer is from 0 to one less than the number of layers.
	FanSpeed     units.Value
	FanFromLayer int
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
	retract := 0.0
	if spec.Retraction != (units.Value{}) {
		if retract, err = positive(spec.Retraction, units.Length, units.Millimeter, "the retraction"); err != nil {
			return err
		}
	}
	nozzle, err := celsius(spec.NozzleTemperature, "the nozzle temperature")
	if err != nil {
		return err
	}
	bed, err := celsius(spec.BedTemperature, "the bed temperature")
	if err != nil {
		return err
	}
	fan := 0.0
	if spec.FanSpeed != (units.Value{}) {
		if spec.FanSpeed.Kind() != units.Dimensionless {
			return fmt.Errorf("export: G-code: %w: the fan speed must be dimensionless", decad.ErrUnitKind)
		}
		fan = spec.FanSpeed.Base()
		if !(fan > 0 && fan <= 1) {
			return fmt.Errorf("export: G-code: %w: the fan speed must lie in (0, 1], got %g", decad.ErrDegenerate, fan)
		}
		if spec.FanFromLayer < 0 || spec.FanFromLayer >= layers {
			return fmt.Errorf("export: G-code: %w: the fan layer %d is not one of the %d layers", decad.ErrDegenerate, spec.FanFromLayer, layers)
		}
	}
	if spec.SolidLayers < 0 {
		return fmt.Errorf("export: G-code: %w: the solid layers are negative", decad.ErrDegenerate)
	}
	perMillimetre := layer * line / (math.Pi * filament * filament / 4)
	var out bytes.Buffer
	fmt.Fprintf(&out, "; perimeters of a %d layer print, %g mm layers\nG21\nG90\nM82\nG92 E0\n", layers, layer)
	if bed > 0 {
		fmt.Fprintf(&out, "M140 S%.0f\n", bed)
	}
	if nozzle > 0 {
		fmt.Fprintf(&out, "M104 S%.0f\n", nozzle)
	}
	if bed > 0 {
		fmt.Fprintf(&out, "M190 S%.0f\n", bed)
	}
	if nozzle > 0 {
		fmt.Fprintf(&out, "M109 S%.0f\n", nozzle)
	}
	if spec.StartCode != "" {
		out.WriteString(strings.TrimRight(spec.StartCode, "\n") + "\n")
	}
	extruded := 0.0
	printed := false // an extruding move has been written, so a travel after it retracts
	travel := func(x, y, z float64) {
		if retract > 0 && printed {
			fmt.Fprintf(&out, "G1 E%.5f F%.1f\n", extruded-retract, feed*60)
		}
		fmt.Fprintf(&out, "G0 X%.4f Y%.4f Z%.4f\n", x, y, z)
		if retract > 0 && printed {
			fmt.Fprintf(&out, "G1 E%.5f F%.1f\n", extruded, feed*60)
		}
	}
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
		if fan > 0 && i == spec.FanFromLayer {
			fmt.Fprintf(&out, "M106 S%.0f\n", math.Round(fan*255))
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
			travel(loop[0].X, loop[0].Y, top)
			for k := 1; k <= len(loop); k++ {
				from, to := loop[k-1], loop[k%len(loop)]
				extruded += to.Sub(from).Len() * perMillimetre
				fmt.Fprintf(&out, "G1 X%.4f Y%.4f E%.5f F%.1f\n", to.X, to.Y, extruded, feed*60)
				printed = true
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
			travel(seg.a.X, seg.a.Y, top)
			extruded += seg.b.Sub(seg.a).Len() * perMillimetre
			fmt.Fprintf(&out, "G1 X%.4f Y%.4f E%.5f F%.1f\n", seg.b.X, seg.b.Y, extruded, feed*60)
			printed = true
		}
	}
	if fan > 0 {
		out.WriteString("M107\n")
	}
	if spec.EndCode != "" {
		out.WriteString(strings.TrimRight(spec.EndCode, "\n") + "\n")
	}
	if nozzle > 0 {
		out.WriteString("M104 S0\n")
	}
	if bed > 0 {
		out.WriteString("M140 S0\n")
	}
	if n, err := w.Write(out.Bytes()); err != nil {
		return fmt.Errorf("export: G-code write failed: %w", err)
	} else if n != out.Len() {
		return fmt.Errorf("export: G-code write failed: %w", io.ErrShortWrite)
	}
	return nil
}

// celsius is a temperature in degrees Celsius, zero for the unset value. A temperature of another kind is
// decad.ErrUnitKind, and one that is not above 0 or is 1000 or more, or not finite, decad.ErrDegenerate.
func celsius(t units.AffineValue, what string) (float64, error) {
	if t == (units.AffineValue{}) {
		return 0, nil
	}
	if t.Kind() != units.Temperature {
		return 0, fmt.Errorf("export: G-code: %w: %s must be a temperature", decad.ErrUnitKind, what)
	}
	c, err := t.InAffine(units.Celsius)
	if err != nil {
		return 0, fmt.Errorf("export: G-code: %w: %s: %s", decad.ErrDegenerate, what, err)
	}
	if !(c > 0 && c < 1000) {
		return 0, fmt.Errorf("export: G-code: %w: %s must lie between 0 and 1000 degrees Celsius, got %g", decad.ErrDegenerate, what, c)
	}
	return c, nil
}
