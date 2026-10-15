package decad

import (
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// loftMatchingSectionRecord reads an interior parallel section from a direct
// solid Loft whose two authenticated profile records are identical. Zero
// alignment pairs every outer and hole boundary point with itself, so every
// ruled wall is vertical and every intermediate material region matches.
func loftMatchingSectionRecord(pl loftPayload, plane r3.Frame) (profileRecord, bool) {
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult ||
		pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		pl.ThroughBoreCut || pl.AnalyticBore || pl.Xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.Profile0, pl.Profile1) {
		return profileRecord{}, false
	}
	for _, offset := range pl.Alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.Frame0.U() != x || pl.Frame0.V() != y || pl.Frame0.N() != z ||
		pl.Frame1.U() != x || pl.Frame1.V() != y || pl.Frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.Frame0.Origin().X != pl.Frame1.Origin().X ||
		pl.Frame0.Origin().Y != pl.Frame1.Origin().Y ||
		pl.Frame0.Origin().X != plane.Origin().X ||
		pl.Frame0.Origin().Y != plane.Origin().Y ||
		pl.Plane0.Origin.Z != pl.Frame0.Origin().Z ||
		pl.Plane1.Origin.Z != pl.Frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.Frame0.Origin().Z), proofarith.FloatRat(pl.Frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	return pl.Profile0, true
}
