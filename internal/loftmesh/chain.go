package loftmesh

import (
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

// ValidateChainLoftRecords is docs/loft-design.md Table SL rows SL3, SL4, SL5
// and SL7 over the two authenticated records alone, in §4's stated gate order
// with the loop dimension collapsed: a ChainRecord holds one walk and no holes,
// so P1 and P2 have nothing left to decide and P3's segment-count
// comparison is over the two walks themselves.
//
// It returns each walk's own resolved SegmentWalk list, in recorded order and
// never rotated: P4's modular wrap is exactly what an open walk drops, so
// segment j pairs with segment j and the alignment offset a loop pair carries
// has no counterpart here. Each segment is walked exactly ONCE, in the same
// interleaved order ValidateLoftRecords uses, because walkOf charges the
// free-form work budget on every call.
func ValidateChainLoftRecords(c0, c1 sectionrecord.ChainRecord, pl0, pl1 sectionrecord.PlaneRecord, work0, work1 *freeform.FreeformWork) ([]survey2d.SegmentWalk, []survey2d.SegmentWalk, error) {
	n := len(c0.Segments)
	if n != len(c1.Segments) {
		return nil, nil, fmt.Errorf(
			`%w: the first walk has %d segments and the second %d; a loft has no one-to-one pairing for a segment-count mismatch`,
			decaderr.ErrUnsupported, n, len(c1.Segments))
	}

	walks0 := make([]survey2d.SegmentWalk, n)
	walks1 := make([]survey2d.SegmentWalk, n)
	for j := range n {
		w0, err := boundarywalk.WalkOf(c0.Segments[j], work0)
		if err != nil {
			return nil, nil, err
		}
		w1, err := boundarywalk.WalkOf(c1.Segments[j], work1)
		if err != nil {
			return nil, nil, err
		}
		// SL4 first, in SameKindGate's own three-way form, so a mixed-kind
		// pairing and an opposite-sense circular pairing each keep the sentinel
		// and the wording Loft already gives them. Loop index 0: a chain takes
		// the outer-loop convention throughout (docs/surface-design.md §13.4).
		if err := SameKindGate(c0.Segments[j], c1.Segments[j], 0, j, j); err != nil {
			return nil, nil, err
		}
		// SL7: this increment rules a LineSeg pair alone. A same-kind circular
		// pair is admitted by Table P and staged here, because §5.1's station
		// generator states each COMPUTED station's own position only within its
		// own stationRound, and §16.2's side gate is stated on the two PLANES
		// rather than on a station for exactly that reason.
		if PairTypeOf(c0.Segments[j]) != PairLine {
			return nil, nil, fmt.Errorf(
				`%w: LoftChain rules a LineSeg pair only; segment %d is a curved pair, whose stations have no side proof yet (docs/loft-design.md §16.2)`,
				decaderr.ErrUnsupported, j)
		}
		walks0[j] = w0
		walks1[j] = w1
	}

	if PlanesCoincide(pl0, pl1) {
		return nil, nil, fmt.Errorf(`%w: the two chains lie in the same geometric plane; the ribbon has zero area by construction`, decaderr.ErrDegenerate)
	}
	if err := ChainLoftPlaneSideGate(pl0, pl1); err != nil {
		return nil, nil, err
	}
	return walks0, walks1, nil
}

// ChainLoftPlaneSideGate is docs/loft-design.md §16.2's admission gate, and
// the whole of what states a chain loft's positive side.
//
// A closed loft orients its shell from the signed tetrahedron sum over the
// complete triangle set, walls and both caps (§5). Two open walls assemble no
// closed set, so that sum is anchor-dependent and states nothing. What this
// build states instead is per wall: each cell's LOWER triangle is wound so its
// own normal agrees with T x N0, T the from-walk's chord direction across that
// cell and N0 = U0 x V0 the from-plane's positive normal — the identical
// vector ExtrudeChain's wall over the same recorded segment publishes
// (docs/surface-design.md Table G). That winding agrees with T x N0 exactly
// when the rule vector from a from-station to the paired to-station has a
// strictly positive component along N0, and exactly parallel planes give that
// at EVERY station at one exactly-positive offset, whatever the station's own
// plane-local coordinates are and whether the generator pinned it or computed
// it.
//
// Both signs are exact rationals over the two records' own U, V and Origin
// floats — internal/proof/exact_point.go's proof.XptOf/proof.Xcross/proof.XdotSign, the package's
// take-the-floats-exactly discipline — so the gate rests on no tolerance and
// no residual. It is reject-only: it can refuse a pose, and it never admits
// one on a small number. The rejected alternative is reading the sign of
// (W - V) . N0 at each HELD station, which decides admission from a float
// sitting within its own stationRound of the point the record denotes — an
// admission gate resting on a bound, which CLAUDE.md forbids outright.
//
// The caller reaches this gate only after PlanesCoincide has refused the
// coplanar pose (S5), so a parallel pair that survives that refusal has a
// strictly nonzero offset and the sign below can only be positive or negative.
func ChainLoftPlaneSideGate(pl0, pl1 sectionrecord.PlaneRecord) error {
	n0 := proof.Xcross(proof.XptOf(pl0.U), proof.XptOf(pl0.V))
	n1 := proof.Xcross(proof.XptOf(pl1.U), proof.XptOf(pl1.V))
	cr := proof.Xcross(n0, n1)
	if cr.X.Sign() != 0 || cr.Y.Sign() != 0 || cr.Z.Sign() != 0 {
		return fmt.Errorf(
			`%w: the two chain planes are not exactly parallel, so this evaluator has no stated positive side for the ribbon between them (docs/loft-design.md §16.2)`,
			decaderr.ErrUnsupported)
	}
	if proof.XdotSign(n0, proof.Xsub(proof.XptOf(pl1.Origin), proof.XptOf(pl0.Origin))) <= 0 {
		return fmt.Errorf(
			`%w: the second chain's plane does not lie on the first plane's positive side, so this evaluator has no stated positive side for the ribbon between them (docs/loft-design.md §16.2)`,
			decaderr.ErrUnsupported)
	}
	return nil
}

// ChainLoftStations places the two station chains docs/loft-design.md §16.5
// describes: one station per recorded segment plus the walk's own terminal
// point, so an n-segment walk carries n+1 stations and the cell walk runs j to
// j+1 with no wrap.
//
// A loop's chain carries only each segment's OWN station because the next
// segment's first station, or the loop's wrap, supplies the one it shares. An
// open walk has no wrap, so its last segment's end point is carried
// explicitly — the identical drop buildChainSides already makes for a chain
// prism's n+1 rim posts (docs/surface-design.md §13.4).
//
// The returned stationRound is the MAX over every station of the displacement
// §5.2 proves for it, each read through the same WalkEndPlaneDelta the closed
// build's own LineSeg arm reads (LineCellPoints). A trimmed LineSeg
// station lands on walkOf's float lerp2 endpoint rather than the exact
// rational the record denotes, so this term is not zero merely because the
// pairing is straight.
func ChainLoftStations(walks0, walks1 []survey2d.SegmentWalk) ([]sectionrecord.Point2, []sectionrecord.Point2, float64, error) {
	n := len(walks0)
	v := make([]sectionrecord.Point2, 0, n+1)
	w := make([]sectionrecord.Point2, 0, n+1)
	round := 0.0
	for j := range n {
		s0, s1, _, _, cellRound, err := LineCellPoints(walks0[j], walks1[j])
		if err != nil {
			return nil, nil, 0, err
		}
		v = append(v, s0...)
		w = append(w, s1...)
		round = math.Max(round, cellRound)
	}
	terminal := math.Max(WalkEndPlaneDelta(walks0[n-1].EndBound), WalkEndPlaneDelta(walks1[n-1].EndBound))
	if proofbound.IsNonFinite(terminal) {
		return nil, nil, 0, ErrLoftStationDisplacementUnderivable
	}
	v = append(v, sectionrecord.Point2{U: walks0[n-1].EndU, V: walks0[n-1].EndV})
	w = append(w, sectionrecord.Point2{U: walks1[n-1].EndU, V: walks1[n-1].EndV})
	return v, w, math.Max(round, terminal), nil
}
