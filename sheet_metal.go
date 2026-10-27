package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// defaultKFactor is the neutral-axis position a SheetMetalSpec with no KFactor takes, the usual value for
// air-bent mild steel.
const defaultKFactor = 0.44

// SheetMetalSpec describes a bent strip of sheet metal (agent-docs/sheet-metal-design.md): flat legs joined
// by circular bends of one inside radius, all of one thickness, a constant Width across.
type SheetMetalSpec struct {
	// Thickness of the sheet, a positive length.
	Thickness units.Value
	// InnerRadius is the radius of every bend on its inside face, a positive length.
	InnerRadius units.Value
	// Width of the strip across the bends, the length of every bend line, a positive length.
	Width units.Value
	// KFactor places the neutral axis, as a fraction of the thickness from the inside face, in (0, 1]. The
	// zero Value takes 0.44. It sets the flat pattern's length and not the part's shape.
	KFactor units.Value
	// Legs are the lengths of the flat portions, in order, each a positive length measured from one bend's end
	// to the next bend's start (the first leg from the strip's end).
	Legs []units.Value
	// Bends are the angles the strip turns between neighbouring legs, one fewer than Legs: a positive angle
	// turns it counter-clockwise in the strip's plane and a negative one clockwise, each nonzero and less than
	// a half turn.
	Bends []units.Value
}

// SheetMetalReport states the flat pattern of a strip [SheetMetal] built.
type SheetMetalReport struct {
	// FlatLength is the length of the blank the strip is cut from: the legs plus each bend's allowance.
	FlatLength units.Value
	// BendAllowances are the lengths of the neutral axis through each bend, angle times (InnerRadius +
	// KFactor·Thickness), in the order of Bends.
	BendAllowances []units.Value
}

// SheetMetal builds a bent strip as one solid in d and reports its flat pattern (agent-docs/sheet-metal-design.md).
// The strip lies in the XY plane with its first leg starting at the origin along +X, and is Width deep along +Z;
// move it with [Transform]. Each bend is a circular arc about a centre on the side it turns toward, the inside face
// of radius InnerRadius and the outside face InnerRadius + Thickness, so the legs meet their bends tangentially
// and the solid is the extrusion of the thickened centreline.
//
// Every length is positive and every angle nonzero and under a half turn (ErrDegenerate for a zero or
// a missing one, ErrNegativeMagnitude for a negative length, ErrUnitKind for a wrong kind, ErrNotFinite); there is
// one more leg than bends; a KFactor outside (0, 1] is ErrDegenerate. A strip whose legs and bends cross
// themselves, so that its outline is not one simple loop, is ErrDegenerate. A nil context or document is
// ErrDegenerate.
func SheetMetal(ctx context.Context, d *Document, spec SheetMetalSpec) (*Body, SheetMetalReport, error) {
	if ctx == nil {
		return nil, SheetMetalReport{}, fmt.Errorf(`%w: a nil context cannot control a sheet metal strip`, ErrDegenerate)
	}
	if d == nil {
		return nil, SheetMetalReport{}, fmt.Errorf(`%w: a sheet metal strip needs a document`, ErrDegenerate)
	}
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
	t, err := length(spec.Thickness, "the sheet thickness")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	r, err := length(spec.InnerRadius, "the bend radius")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	width, err := length(spec.Width, "the strip width")
	if err != nil {
		return nil, SheetMetalReport{}, err
	}
	k := defaultKFactor
	if spec.KFactor != (units.Value{}) {
		if spec.KFactor.Kind() != units.Dimensionless {
			return nil, SheetMetalReport{}, fmt.Errorf(`%w: the K-factor must be dimensionless`, ErrUnitKind)
		}
		if k = spec.KFactor.Base(); !(k > 0 && k <= 1) {
			return nil, SheetMetalReport{}, fmt.Errorf(`%w: the K-factor must lie in (0, 1], got %g`, ErrDegenerate, k)
		}
	}
	if len(spec.Legs) == 0 || len(spec.Bends) != len(spec.Legs)-1 {
		return nil, SheetMetalReport{}, fmt.Errorf(`%w: a strip needs one more leg than bends, got %d legs and %d bends`,
			ErrDegenerate, len(spec.Legs), len(spec.Bends))
	}
	legs := make([]float64, len(spec.Legs))
	for i, v := range spec.Legs {
		if legs[i], err = length(v, fmt.Sprintf("leg %d", i)); err != nil {
			return nil, SheetMetalReport{}, err
		}
	}
	bends := make([]float64, len(spec.Bends))
	for i, v := range spec.Bends {
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
	body, err := Extrude(ctx, d, s, outline, Distance{D: units.Millimeters(width), Dir: Along})
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
