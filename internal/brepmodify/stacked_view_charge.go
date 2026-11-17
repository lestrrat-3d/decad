package brepmodify

import (
	"math"
	"strings"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// stackedViewCharge lists, by index in topobuild.Faces order, the swept
// circular walls of b whose view keeps its Cylinder tag and states a bound on
// it, with that bound. It is nil when the record bp is exact (no section
// displacement and no carrier charge) or when the bound is not a finite
// nonnegative number.
//
// A swept circular wall is an arc pinned about its recorded centre with its
// radius read from its first vertex, so the circle it lies on has a centre
// offset plus radius difference from the circle it denotes within the record's
// section displacement, which holds a holed union's CarrierCharge (general-boolean
// §3 "A1 as a brep": Displacements, and the width band's |rj − rk| + |cj − ck| +
// 2·|ck − c0|). That bounds the two circles apart all the way round. The view keeps its Cylinder tag and
// states that bound as the tag's AxialDelta; the retained edge blend charges
// it (R2) or refuses it.
func stackedViewCharge(b *topo.Body, bp breprecord.Payload) map[int]float64 {
	if bp.SectionDelta() == 0 && bp.CarrierCharge == 0 {
		return nil
	}
	w := max(bp.SectionDelta(), bp.CarrierCharge)
	if !(w > 0) || math.IsInf(w, 0) {
		return nil
	}
	var out map[int]float64
	for i, f := range topobuild.Faces(b) {
		if !chargedViewWall(b, f) {
			continue
		}
		if out == nil {
			out = map[int]float64{}
		}
		out[i] = w
	}
	return out
}

// chargedViewWall reports whether f is a swept circular wall of b: not a planar
// record face, a Cylinder surface, and a wall(k) origin under b's own producer.
func chargedViewWall(b *topo.Body, f *topo.Face) bool {
	if recordPlanarFace(b, f) {
		return false
	}
	if _, ok := f.Surface.(surfacegeom.Cylinder); !ok {
		return false
	}
	for _, origin := range f.Origins {
		if origin.Producer == b.Origin.Producer && strings.HasPrefix(origin.Role, "wall(") {
			return true
		}
	}
	return false
}
