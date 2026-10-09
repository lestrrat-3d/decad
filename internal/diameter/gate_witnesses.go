package diameter

import (
	"math"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tolerance"
	"github.com/lestrrat-3d/r3"
)

// GatePoints holds witnesses within Allow of points on the measured geometry.
type GatePoints struct {
	Points []r3.Vec
	Allow  float64
}

// Join adds another witness set and takes the larger allowance.
func (g *GatePoints) Join(other GatePoints) {
	g.Points = append(g.Points, other.Points...)
	g.Allow = math.Max(g.Allow, other.Allow)
}

// Diameter gives a lower bound on the measured geometry's diameter by
// shrinking the held points' lower pair distance by twice Allow.
func (g GatePoints) Diameter(budget *proofbound.WorkBudget) (float64, bool, error) {
	d, ok, err := PointsWithBudget(budget, g.Points)
	if err != nil || !ok {
		return 0, false, err
	}
	d, ok = LowerForDisplacement(d, g.Allow)
	return d, ok, nil
}

// RevolveWitnessInput is the recorded sweep and placement read by a revolve
// body's diameter gate.
type RevolveWitnessInput struct {
	Phi0, Phi1   float64
	Den0, Den1   revolveangle.Angle
	Loops        []sectionrecord.LoopRecord
	Lift         revolvemesh.RevolveLift
	AxisBound    revolvemesh.AxisBound
	Axis         revolveaxis.Frame
	Transform    r3.Transform
	SectionDelta float64
}

// RevolveWitnesses sweeps each meridian station to the gate angles and
// bounds its gap from the recorded surface point it denotes.
//
// The farthest pair on two circles grows with their angular separation up to
// half a turn. RevolveGateAngles therefore selects the two sweep ends for a
// sweep of at most half a turn, or a half-turn pair otherwise. Every station
// is swept to those angles. SweptPointGap compares each held point with the
// recorded station rotated to its denoted angle, including the station's own
// gap, SectionDelta, the axis, frame, and placement rounding. A missing
// angle or unbounded gap withholds the reading.
func RevolveWitnesses(budget *proofbound.WorkBudget, input RevolveWitnessInput) (GatePoints, bool, error) {
	angles, ok := RevolveGateAngles(input.Phi0, input.Phi1, input.Den0, input.Den1)
	if !ok {
		return GatePoints{}, false, nil
	}
	stations, ok, err := SectionStations(budget, input.Loops, freeform.NewFreeformWork())
	if err != nil || !ok {
		return GatePoints{}, false, err
	}
	basis := input.Lift.Basis()
	points := make([]r3.Vec, 0, len(stations)*len(angles))
	allow := 0.0
	for _, s := range stations {
		uv := proofbound.WalkEndBound{
			U: proofbound.AbsSumUpper(s.Bound.U, input.SectionDelta),
			V: proofbound.AbsSumUpper(s.Bound.V, input.SectionDelta),
		}
		z, rho := input.Axis.ToAxis(s.U, s.V)
		for _, a := range angles {
			if err := budget.Step(); err != nil {
				return GatePoints{}, false, err
			}
			held := input.Lift.Point(basis, input.Transform, z, rho, a.Phi)
			gap := input.Lift.SweptPointGap(input.AxisBound, input.Transform, s.U, s.V, uv, a.Sin, a.Cos, held)
			if !proofbound.FiniteVec(held) || !tolerance.UsableMagnitude(gap) {
				return GatePoints{}, false, nil
			}
			allow = math.Max(allow, gap)
			points = append(points, held)
		}
	}
	return GatePoints{Points: points, Allow: allow}, true, nil
}

// CapArc is one trimmed cap contour arc whose stations may witness a body's
// diameter. Allow includes its radial and angular displacement.
type CapArc struct {
	Center, Start, End sectionrecord.Point2
	Level, Allow       float64
}

// CapArcWitness reads a circular patch's cap contour arc. Other patches and
// unbounded contour displacements return false.
//
// The recorded cap arc runs counter-clockwise from CapA to CapB around the
// patch center. Its radius differs from the held offset radius by at most
// Held.CapRadius, and the offset radius differs from the denoted radius by at
// most contour. Each cap endpoint is within contour of its corner foot. With
// rMin = CapRadius - max(Held.CapRadius, contour), that endpoint gap widens
// its angle by at most beta = (π/2)·contour/rMin. The returned allowance is
// the radial terms plus (CapRadius + Held.CapRadius)·beta. A held sweep that
// disagrees with the recorded window by more than 1e-9 rad is refused.
func CapArcWitness(g capband.Patch, contour float64) (CapArc, bool) {
	if !g.Circular || g.WholeTurn || g.SideRadius <= 0 || !(g.CapTh1 > g.CapTh0) {
		return CapArc{}, false
	}
	center := sectionrecord.Point2{U: g.CU, V: g.CV}
	if g.CapA == center || g.CapB == center {
		return CapArc{}, false
	}
	a0 := math.Atan2(g.CapA.V-g.CV, g.CapA.U-g.CU)
	sweep := math.Mod(math.Atan2(g.CapB.V-g.CV, g.CapB.U-g.CU)-a0, 2*math.Pi)
	if sweep <= 0 {
		sweep += 2 * math.Pi
	}
	if math.Abs(sweep-(g.CapTh1-g.CapTh0)) > 1e-9 {
		return CapArc{}, false
	}
	rMin := math.Nextafter(g.CapRadius-math.Max(g.Held.CapRadius, contour), 0)
	if !(rMin > contour) || proofbound.IsNonFinite(rMin) {
		return CapArc{}, false
	}
	beta := proofbound.DivUpper(proofbound.ProductUpper(math.Nextafter(math.Pi/2, math.Inf(1)), contour), rMin)
	radial := proofbound.AbsSumUpper(g.Held.CapRadius, contour)
	allow := proofbound.AbsSumUpper(radial,
		proofbound.ProductUpper(proofbound.AbsSumUpper(g.CapRadius, g.Held.CapRadius), beta))
	if !tolerance.UsableMagnitude(allow) {
		return CapArc{}, false
	}
	return CapArc{Center: center, Start: g.CapA, End: g.CapB, Level: g.CapZ, Allow: allow}, true
}
