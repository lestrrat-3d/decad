package tessellation

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// CapBlendSetback holds the measured planar and axial distances of a band.
type CapBlendSetback struct {
	AxialUpper, Dc, DcDelta, Ds, DsDelta float64
}

// CapBlendLocusInput identifies one corner and the records needed for its bound.
type CapBlendLocusInput struct {
	Setback   CapBlendSetback
	Segments  []sectionrecord.CurveSegment
	Walks     []survey2d.SideWalk
	Corner    int
	Join      CapBlendJoin
	FootDelta float64
}

// CapBlendCornerLocusGap is how far a MITER corner's built ruling — an Edge
// tagged Line3, straight from the cap-level foot down to the original corner —
// can sit from the conic miter locus it stands for
// (docs/tessellation-reach-design.md §7's locusGap).
//
// The locus is a curve of proven length at most L between two endpoints c*
// apart, so it lies inside the ellipse with those two endpoints as foci and
// major axis L, whose semi-minor axis is sqrt(L² − c*²)/2. L is the same
// subdivided bound capSlantEdge charges the ruling's own length against
// (capband.MiterLocusUpper). The semi-minor axis only grows as the focal distance
// shrinks, so c must be a proven LOWER bound on c*, the distance between the
// locus's own ends: the denoted corner and the denoted foot, the stated side
// setback ds* apart along the sweep. The held chord from the corner to the
// held foot m at the held ds is not one. The foot sits within the band's
// contour displacement footDelta of the denoted foot, the corner within the
// bound reaching both neighbours' denoted ends there of the denoted corner,
// and ds within dsDelta of ds*, so c is the held chord, its square taken
// exactly over the rationals and its root rounded down, less those three
// (CapBlendCornerChordSqLower). A setback stated in millimetres on a
// recorded section with exact feet subtracts nothing.
//
// It is zero where both loci are affine in the offset amount — a line-line
// miter, every reflex corner's own two feet, which ride one carrier each, and
// every G1 join (modify §7), whose foot is v + s·dc·n̂ — and it is the SAME
// number at either cap, since a loop chamfered on both caps takes one setback
// pair on both (resolveCapSetbacks) and the two readings then differ only in
// the sign of an axial span both take the magnitude of. The locus runs dc in
// the plane and ds along the sweep (docs/modify-reach-design.md §8.3.1). A
// sub-range whose speed cannot be enclosed answers +Inf, which refuses.
func CapBlendCornerLocusGap(budget *proofbound.WorkBudget, in CapBlendLocusInput) (float64, error) {
	n := len(in.Walks)
	prev, cur := in.Walks[(in.Corner+n-1)%n], in.Walks[in.Corner]
	if in.Join.G1 || (!prev.IsCircular() && !cur.IsCircular()) {
		return 0, nil
	}
	locus, ok, err := capband.MiterLocusUpper(budget, prev, cur, in.Join.VU, in.Join.VV,
		in.Setback.AxialUpper, in.Setback.Dc, in.Setback.DcDelta)
	if err != nil {
		return 0, err
	}
	if !ok || proofbound.IsNonFinite(locus) {
		return 0, fmt.Errorf(`%w: a cap-loop chamfer's miter ruling states no enclosure of the locus it stands for, so this mesh can publish no displacement bound for the patches that share it`, decaderr.ErrUnsupported)
	}
	chordSqDown, err := CapBlendCornerChordSqLower(in)
	if err != nil {
		return 0, err
	}
	diff := proofbound.UpRound(proofbound.ProductUpper(locus, locus) - chordSqDown)
	if diff <= 0 {
		return 0, nil
	}
	return proofbound.UpRound(math.Sqrt(diff) / 2), nil
}

// CapBlendCornerChordSqLower is the proven lower bound on c*², the squared
// distance CapBlendCornerLocusGap's ellipse takes between the locus's ends:
// the held chord from the corner (j.vU, j.vV) at level 0 to the held foot m
// at the held ds, its square exact and its root rounded down, less the foot's
// footDelta, the corner's own bound and dsDelta. The corner is walk i's held
// start, where walk i−1 ends, and its bound reaches the points both
// neighbours' records denote there (boundarywalk.JunctionStartBound), so an
// arc's natural t = 1 end, whose held End sits off Start's radius, is
// reached. segs are the recorded segments the walks' Segs index.
func CapBlendCornerChordSqLower(in CapBlendLocusInput) (float64, error) {
	n := len(in.Walks)
	prev, cur := in.Walks[(in.Corner+n-1)%n], in.Walks[in.Corner]
	chordSq := proofarith.RatSquaredDistance3(in.Join.M.U, in.Join.M.V, in.Setback.Ds,
		in.Join.VU, in.Join.VV, 0)
	chordSqDown, exact := chordSq.Float64()
	if !exact {
		chordSqDown = math.Nextafter(chordSqDown, math.Inf(-1))
	}
	cornerDelta := proofbound.WalkEndBoundAllow(boundarywalk.JunctionStartBound(
		in.Segments[prev.Segs[len(prev.Segs)-1]], prev.SegmentWalk,
		in.Segments[cur.Segs[0]], cur.SegmentWalk))
	if proofbound.IsNonFinite(cornerDelta) || proofbound.IsNonFinite(in.FootDelta) {
		return 0, fmt.Errorf(`%w: a cap-loop chamfer's miter corner states no bound on its own ends, so this mesh can publish no displacement bound for the patches that share its ruling`, decaderr.ErrUnsupported)
	}
	if shift := proofbound.AbsSumUpper(in.FootDelta, cornerDelta, in.Setback.DsDelta); shift > 0 {
		chordLower := freeform.DownRound(proofbound.RatSqrtDown(chordSq) - shift)
		chordSqDown = 0
		if chordLower > 0 {
			chordSqDown = freeform.DownRound(chordLower * chordLower)
		}
	}
	return chordSqDown, nil
}
