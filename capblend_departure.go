package decad

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// capPatchNormalAllow adapts built cap-band faces to the exact departure
// bounds in internal/capband/departure.go.

// ruledNormalAllowUnbounded is the trivial bound on the distance between two
// unit vectors: any two of them differ by at most 2. It is what
// capPatchNormalAllow returns wherever its own derivation degenerates — an
// honest "this reading proves nothing" rather than a smaller number nothing
// supports. Every consumer treats it as undecided rather than as a width.
const ruledNormalAllowUnbounded = 2.0

type capDirectrixRef = capband.DirectrixRef

type capPatchBuilt = capband.PatchBuilt

// capPatchNormalAllow is the file's answer for one patch: the proven bound on
// how far the RULED surface the build assembles can carry a normal differing
// from the surface the patch publishes.
//
// Both patch kinds answer from their own held corners, and neither is exempt.
// A straight wall's offset family is affine in the offset amount, so the
// surface a flat patch denotes is a plane — but the corners the build emits are
// each rounded once more, and the tag is fixed through three of the four, so
// the built quad still leaves the plane by an amount only a world-space reading
// of those four corners states. How far the built patch turns from the denoted
// one is a separate term (capband.DenotedNormalAllow), which setPatchReadings
// adds.
func capPatchNormalAllow(f *Face, g capPatchGeom, b capPatchBuilt) float64 {
	departure := capPlaneDeparture
	if g.Circular {
		departure = capPatchDeparture
	}
	allow, ok := departure(f, b)
	if !ok || proofbound.IsNonFinite(allow) || allow < 0 {
		return ruledNormalAllowUnbounded
	}
	return math.Min(ruledNormalAllowUnbounded, allow)
}

// capPlaneDeparture reads the published plane tag before bounding its built patch.
func capPlaneDeparture(f *Face, b capPatchBuilt) (float64, bool) {
	tag, okTag := f.surface.(Plane)
	if !okTag {
		return 0, false
	}
	return capband.PlaneDeparture(tag.Frame.U(), tag.Frame.V(), b)
}

// capPatchDeparture reads the published cone tag before bounding its built patch.
func capPatchDeparture(f *Face, b capPatchBuilt) (float64, bool) {
	sinH, cosH, originVec, axisVec, ok := coneTagTerms(f)
	if !ok {
		return 0, false
	}
	tagRadius, okR := coneTagRadius(f)
	if !okR {
		return 0, false
	}
	return capband.ConeDeparture(sinH, cosH, originVec, axisVec, tagRadius, b)
}

// coneTagRadius is the radius a circular patch's own tag carries at its origin
// — zero for every tag the cap band builds, since coneSurface anchors a Cone at
// its apex. It is read rather than assumed, so the signed distance κ this file
// measures stays the tag's own implicit function whatever radius a tag states.
func coneTagRadius(f *Face) (*big.Rat, bool) {
	var value units.Value
	switch s := f.surface.(type) {
	case Cone:
		value = s.Radius
	case Cylinder:
		value = s.Radius
	default:
		return nil, false
	}
	mm, err := value.In(units.Millimeter)
	if err != nil {
		return nil, false
	}
	r := proofarith.FloatRat(mm)
	if r == nil {
		return nil, false
	}
	return r, true
}

// capBuiltPatch names one band patch's built ruled surface from the two
// published directrix edges and the boundary vertices that pair them: ruling k
// joins sideEnds[k] to capEnds[k]. A directrix this evaluator cannot read
// leaves its own reference empty, which either departure derivation refuses and
// the caller publishes the trivial bound for, rather than a number no reading
// supports.
func capBuiltPatch(sideEdge, capEdge *Edge, sideEnds, capEnds []*Vertex) capPatchBuilt {
	return capPatchBuilt{
		SideDir: capDirectrixFromEdge(sideEdge, sideEnds...),
		CapDir:  capDirectrixFromEdge(capEdge, capEnds...),
	}
}

// capBuiltApexPatch is the same for a reflex corner's apex patch, whose side
// directrix is the single original corner VERTEX the rulings all leave: a
// circle of radius zero, which capPatchDeparture admits only where that vertex
// is the tag's own origin held for held.
func capBuiltApexPatch(apex *Vertex, capEdge *Edge, capEnds []*Vertex) capPatchBuilt {
	capped := capDirectrixFromEdge(capEdge, capEnds...)
	if apex == nil {
		return capPatchBuilt{CapDir: capped}
	}
	ends := make([]r3.Vec, len(capped.Ends))
	for i := range ends {
		ends[i] = apex.position
	}
	return capPatchBuilt{
		SideDir: capDirectrixRef{Center: apex.position, Ends: ends},
		CapDir:  capped,
	}
}

// capDirectrixFromEdge reads one published directrix edge. Only the three curve
// kinds a band's directrix is ever built as are admitted — the two circular ones
// a wall over an arc gives, and the `Line3` a straight wall gives, which carries
// no circle and is read through its ruling endpoints alone; anything else leaves
// the patch without a stated built surface and the caller publishes the trivial
// bound.
func capDirectrixFromEdge(e *Edge, ends ...*Vertex) capDirectrixRef {
	var out capDirectrixRef
	var radius units.Value
	switch c := e.curve.(type) {
	case Arc3:
		out.Center, out.Axis, radius = c.Center, c.Axis, c.Radius
	case Circle3:
		out.Center, out.Axis, radius = c.Center, c.Axis, c.Radius
	case Line3:
		out.Straight = true
	default:
		return capDirectrixRef{}
	}
	if !out.Straight {
		mm, err := radius.In(units.Millimeter)
		if err != nil {
			return capDirectrixRef{}
		}
		out.Radius = mm
	}
	for _, v := range ends {
		if v == nil {
			return capDirectrixRef{}
		}
		out.Ends = append(out.Ends, v.position)
	}
	return out
}
