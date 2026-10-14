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
	if !pl.authenticatedReconstruction || pl.surfaceResult ||
		pl.constructionDelta != 0 || len(pl.blendSegs) != 0 ||
		pl.throughBoreCut || pl.analyticBore || pl.xform != r3.Identity() ||
		!sectionrecord.IdenticalRecord(pl.profile0, pl.profile1) {
		return profileRecord{}, false
	}
	for _, offset := range pl.alignment {
		if offset != 0 {
			return profileRecord{}, false
		}
	}
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	if pl.frame0.U() != x || pl.frame0.V() != y || pl.frame0.N() != z ||
		pl.frame1.U() != x || pl.frame1.V() != y || pl.frame1.N() != z ||
		plane.U() != x || plane.V() != y || plane.N() != z {
		return profileRecord{}, false
	}
	if pl.frame0.Origin().X != pl.frame1.Origin().X ||
		pl.frame0.Origin().Y != pl.frame1.Origin().Y ||
		pl.frame0.Origin().X != plane.Origin().X ||
		pl.frame0.Origin().Y != plane.Origin().Y ||
		pl.plane0.Origin.Z != pl.frame0.Origin().Z ||
		pl.plane1.Origin.Z != pl.frame1.Origin().Z {
		return profileRecord{}, false
	}
	z0, z1 := proofarith.FloatRat(pl.frame0.Origin().Z), proofarith.FloatRat(pl.frame1.Origin().Z)
	cut := proofarith.FloatRat(plane.Origin().Z)
	if z0 == nil || z1 == nil || cut == nil || cut.Cmp(z0) <= 0 || cut.Cmp(z1) >= 0 {
		return profileRecord{}, false
	}
	return pl.profile0, true
}
