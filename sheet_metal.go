package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// defaultKFactor is the neutral-axis position a strip with no [SheetMetalCommand.KFactor] takes, the usual value for
// air-bent mild steel.
const defaultKFactor = 0.44

// sheetMetalSpec is the request a [SheetMetalCommand] hands to the implementation:
// the constructor arguments and the methods that were called.
type sheetMetalSpec struct {
	thickness, innerRadius, width units.Value
	legs, bends                   []units.Value
	kFactor                       units.Value
	kFactorSet                    bool
	// kFactorSubject names the method a fault of the K-factor points at.
	kFactorSubject string
}

// SheetMetalReport states the flat pattern of a strip [SheetMetal] built.
type SheetMetalReport struct {
	// FlatLength is the length of the blank the strip is cut from: the legs plus each bend's allowance.
	FlatLength units.Value
	// BendAllowances are the lengths of the neutral axis through each bend, angle times (InnerRadius +
	// KFactor·Thickness), in the order of Bends.
	BendAllowances []units.Value
}

// SheetMetal returns the command that builds a bent strip of sheet metal as one solid in d and
// reports its flat pattern (agent-docs/sheet-metal-design.md): flat legs joined by circular bends of one
// inside radius, all of one thickness, a constant width across.
//
// thickness is the thickness of the sheet, innerRadius the radius of every bend on its inside face and
// width the width of the strip across the bends, the length of every bend line, each a positive length.
// legs are the lengths of the flat portions, in order, each a positive length measured from one bend's end
// to the next bend's start (the first leg from the strip's end). bends are the angles the strip turns
// between neighbouring legs, one fewer than legs: a positive angle turns it counter-clockwise in the
// strip's plane and a negative one clockwise, each nonzero and less than a half turn. The command copies
// both slices. [SheetMetalCommand.KFactor] places the neutral axis, and is 0.44 unless called.
//
// The strip lies in the XY plane with its first leg starting at the origin along +X, and is width deep along +Z;
// move it with [Transform]. Each bend is a circular arc about a centre on the side it turns toward, the inside face
// of radius innerRadius and the outside face innerRadius + thickness, so the legs meet their bends tangentially
// and the solid is the extrusion of the thickened centreline.
//
// Every length is positive and every angle nonzero and under a half turn (ErrDegenerate for a zero or
// a missing one, ErrNegativeMagnitude for a negative length, ErrUnitKind for a wrong kind, ErrNotFinite); there is
// one more leg than bends. A strip whose legs and bends cross
// themselves, so that its outline is not one simple loop, is ErrDegenerate. A nil context or document is
// ErrDegenerate.
func SheetMetal(d *Document, thickness, innerRadius, width units.Value, legs, bends []units.Value) *SheetMetalCommand {
	c := &SheetMetalCommand{d: d, spec: sheetMetalSpec{thickness: thickness, innerRadius: innerRadius,
		width: width, legs: slices.Clone(legs), bends: slices.Clone(bends)}}
	if d == nil {
		c.state.Record("d", errNilSheetMetalDocument)
	}
	return c
}

// errNilSheetMetalDocument is the refusal of a nil document, which the
// constructor records because the operation reports it right after the nil ctx check.
var errNilSheetMetalDocument = fmt.Errorf(`%w: a sheet metal strip needs a document`, ErrDegenerate)

// SheetMetalCommand builds a bent strip of sheet metal. Build it with [SheetMetal].
type SheetMetalCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	d     *Document       // stored as given
	spec  sheetMetalSpec  // the constructor arguments, copied, and the methods called
}

// KFactor places the neutral axis, as a fraction of the thickness from the inside face, in (0, 1].
// It sets the flat pattern's length and not the part's shape. Without it the K-factor is 0.44. A value
// outside (0, 1] is ErrDegenerate and one that is not dimensionless ErrUnitKind, returned by Do naming
// KFactor as the subject. A second call replaces the first.
func (c *SheetMetalCommand) KFactor(k units.Value) *SheetMetalCommand {
	c.spec.kFactor, c.spec.kFactorSet = k, true
	return c
}

// sheetMetalResult is the three results of [SheetMetalCommand.Do], which runCommand carries as one value.
type sheetMetalResult struct {
	body   *Body
	report SheetMetalReport
}

// Do builds the strip, or returns the error of the first gate that refuses, with the document
// unchanged. A nil ctx is ErrDegenerate, and a second call returns ErrConsumed.
func (c *SheetMetalCommand) Do(ctx context.Context) (*Body, SheetMetalReport, error) {
	out, err := runCommand(ctx, &c.state, OperationSheetMetal, c.run)
	return out.body, out.report, err
}

func (c *SheetMetalCommand) run(ctx context.Context) (sheetMetalResult, error) {
	spec := c.spec
	spec.kFactorSubject = "KFactor"
	body, report, err := sheetMetal(ctx, c.d, spec)
	return sheetMetalResult{body, report}, err
}

// sheetMetal is the implementation of SheetMetal. ctx is not nil, and the errors it
// returns carry no operation tag, except the refusal of a stored method value
// (agent-docs/command-api-design.md §11).
func sheetMetal(ctx context.Context, d *Document, spec sheetMetalSpec) (*Body, SheetMetalReport, error) {
	length := func(v units.Value, what string) (float64, error) {
		x, err := sectionrecord.MagnitudeIn(v, units.Length, units.Millimeter, what)
		if err != nil {
			return 0, err
		}
		if x == 0 || math.IsInf(x, 0) {
			return 0, fmt.Errorf(`%w: %s must be positive and finite`, ErrDegenerate, what)
		}
		return x, nil
	}
	t, err := length(spec.thickness, "the sheet thickness")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	r, err := length(spec.innerRadius, "the bend radius")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	width, err := length(spec.width, "the strip width")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	k := defaultKFactor
	if spec.kFactorSet {
		if spec.kFactor.Kind() != units.Dimensionless {
			return nil, SheetMetalReport{}, subjectFault(OperationSheetMetal, spec.kFactorSubject,
				fmt.Errorf(`%w: the K-factor must be dimensionless`, ErrUnitKind))
		}
		if k = spec.kFactor.Base(); !(k > 0 && k <= 1) {
			return nil, SheetMetalReport{}, subjectFault(OperationSheetMetal, spec.kFactorSubject,
				fmt.Errorf(`%w: the K-factor must lie in (0, 1], got %g`, ErrDegenerate, k))
		}
	}
	if len(spec.legs) == 0 || len(spec.bends) != len(spec.legs)-1 {
		return nil, SheetMetalReport{}, fmt.Errorf(`%w: a strip needs one more leg than bends, got %d legs and %d bends`,
			ErrDegenerate, len(spec.legs), len(spec.bends))
	}
	legs := make([]float64, len(spec.legs))
	for i, v := range spec.legs {
		if legs[i], err = length(v, fmt.Sprintf("leg %d", i)); err != nil {
			return nil, SheetMetalReport{}, err
		}
	}
	bends := make([]float64, len(spec.bends))
	for i, v := range spec.bends {
		if v.Kind() != units.Angle {
			return nil, SheetMetalReport{}, fmt.Errorf(`%w: bend %d must be an angle`, ErrUnitKind, i)
		}
		a := v.Base()
		if math.IsNaN(a) || math.IsInf(a, 0) {
			return nil, SheetMetalReport{}, fmt.Errorf(`%w: bend %d is not finite`, ErrNotFinite, i)
		}
		if a == 0 || math.Abs(a) >= math.Pi {
			return nil, SheetMetalReport{}, fmt.Errorf(`%w: bend %d must be nonzero and under a half turn`, ErrDegenerate, i)
		}
		bends[i] = a
	}

	// The centreline's stations: each leg's start and end with the heading along it.
	type vec struct{ x, y float64 }
	type leg struct {
		start, end vec
		heading    float64
	}
	var run []leg
	at, heading := vec{}, 0.0
	rc := r + t/2 // the centreline's bend radius
	var centres []vec
	for i, length := range legs {
		end := vec{at.x + length*math.Cos(heading), at.y + length*math.Sin(heading)}
		run = append(run, leg{start: at, end: end, heading: heading})
		if i == len(bends) {
			break
		}
		turn := bends[i]
		side := 1.0 // the centre lies to the left of a left turn
		if turn < 0 {
			side = -1
		}
		centre := vec{end.x - side*rc*math.Sin(heading), end.y + side*rc*math.Cos(heading)}
		centres = append(centres, centre)
		// Rotate the radius from the centre to the leg's end by the turn.
		rx, ry := end.x-centre.x, end.y-centre.y
		sin, cos := math.Sincos(turn)
		at = vec{centre.x + rx*cos - ry*sin, centre.y + rx*sin + ry*cos}
		heading += turn
	}

	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	point := func(p vec, heading, offset float64) *sketch.Point {
		pt := s.CreatePoint(p.x-offset*math.Sin(heading), p.y+offset*math.Cos(heading))
		s.Fix(pt)
		return pt
	}
	// The strip's left and right faces at each leg's two ends; the face a bend turns toward is the inner one.
	var lefts, rights [][2]*sketch.Point
	for _, l := range run {
		lefts = append(lefts, [2]*sketch.Point{point(l.start, l.heading, t/2), point(l.end, l.heading, t/2)})
		rights = append(rights, [2]*sketch.Point{point(l.start, l.heading, -t/2), point(l.end, l.heading, -t/2)})
	}
	centrePoint := func(i int) *sketch.Point {
		pt := s.CreatePoint(centres[i].x, centres[i].y)
		s.Fix(pt)
		return pt
	}
	for i := range run {
		s.CreateLine(lefts[i][0], lefts[i][1])
		s.CreateLine(rights[i][0], rights[i][1])
	}
	for i, turn := range bends {
		c := centrePoint(i)
		for _, side := range [][2]*sketch.Point{{lefts[i][1], lefts[i+1][0]}, {rights[i][1], rights[i+1][0]}} {
			if turn > 0 {
				s.CreateArc(c, side[0], side[1])
			} else {
				s.CreateArc(c, side[1], side[0])
			}
		}
	}
	last := len(run) - 1
	s.CreateLine(lefts[0][0], rights[0][0])
	s.CreateLine(lefts[last][1], rights[last][1])
	if _, err := s.Solve(ctx); err != nil {
		return nil, SheetMetalReport{}, err
	}
	var outline *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid {
			if outline != nil {
				return nil, SheetMetalReport{}, fmt.Errorf(`%w: the strip's outline is more than one loop`, ErrDegenerate)
			}
			outline = p
		}
	}
	if outline == nil {
		return nil, SheetMetalReport{}, fmt.Errorf(`%w: the strip's outline is not one simple loop`, ErrDegenerate)
	}
	body, err := extrudeTool(ctx, d, s, outline, Distance{D: units.Millimeters(width), Dir: Along}, extrudeSettings{})
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	flat := 0.0
	for _, l := range legs {
		flat += l
	}
	report := SheetMetalReport{}
	for _, a := range bends {
		allowance := math.Abs(a) * (r + k*t)
		flat += allowance
		report.BendAllowances = append(report.BendAllowances, units.Millimeters(allowance))
	}
	report.FlatLength = units.Millimeters(flat)
	return body, report, nil
}
