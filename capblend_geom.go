package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/freeform"

	"github.com/lestrrat-3d/decad/internal/survey2d"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file builds the capBlendPayload topology (docs/modify-reach-design.md
// §8.3, Table BX row BX3): the trimmed prism side walls, the cap faces (a
// mix of unchanged loops and offset "cap contour" loops), and the chamfer
// band patches between them — a Plane for a line wall, a Cone for a circular
// wall, and a Cone whose apex is the original corner for a reflex corner's
// extra offset arc.

// capOffsetJoins returns the shared Shell offset joins after checking the
// cap band's circular walls. The caller pairs each join with its source wall.
func capOffsetJoins(budget *proofbound.WorkBudget, cl cornerLoop, d float64) ([]cornerJoin, error) {
	walks := cl.walks
	n := len(walks)
	if n == 0 {
		return nil, fmt.Errorf(`%w: a cap loop holds no walks`, ErrDegenerate)
	}
	for _, w := range walks {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return nil, err
		}
		if w.IsCircular() {
			if _, err := capBandRadius(w, d); err != nil {
				return nil, err
			}
		}
	}
	return offsetJoinsBudget(budget, walks, 1, d)
}

// capBandRadius is the ONE place a circular cap-band wall's cap-level offset
// radius is resolved, and it proves that the offset the caller asked for
// SURVIVED float64 at this wall's own scale before any patch is built from it.
// It is SX13's RADIAL half; the axial half — the same collapse in the sweep
// direction, where the band's side level rounds back onto its own cap level —
// is a fact about the sweep interval alone and is decided once per chamfered
// cap in capblend.go's requireCapBlendLevelsSeparate.
//
// offsetRadius's own refusal is the empty one: an inward offset that reaches or
// passes the centre leaves no circle at all (errOffsetDrop). The second refusal
// is this one, and it is the opposite failure — an offset so small RELATIVE to
// the radius that `R -/+ d` rounds back onto `R` itself. The band's patch is a
// cone by construction (its two directrices are circles of different radii), and
// a cone whose two stored radii are bit-identical is a CYLINDER: the taper the
// chamfer exists to create is gone, so every reader of that surface — the DX7
// undercut survey above all, which asks exactly about the taper — is answered by
// a shape the caller never asked for. Substituting it is a wrong answer, not a
// coarse one, so the call refuses. The body exists (the cone has a real, if
// tiny, taper) and this evaluator cannot state its carrier in float64 at that
// scale, which is §4's ErrUnsupported side of the existence test (Table SX row
// SX13).
func capBandRadius(w survey2d.SideWalk, d float64) (float64, error) {
	r, ok := offsetRadius(w, 1, d)
	if !ok {
		return 0, errOffsetDrop
	}
	if r == w.Radius {
		return 0, fmt.Errorf(`%w: the chamfer setback %v mm is below the float64 spacing of a circular wall's own radius %v mm, so the cap contour's radial change rounds away and the band's cone patch cannot be told from a cylinder; a wider setback or a smaller radius states a chamfer this evaluator can build`, ErrUnsupported, d, w.Radius)
	}
	return r, nil
}

// capWallFoot returns the offset segment's own (start, end) feet for wall i,
// exactly as offsetLoopBudget's per-wall trim does.
func capWallFoot(joins []cornerJoin, i, n int) (Point2, Point2) {
	j0 := joins[i]
	start := j0.m
	if j0.arc {
		start = j0.pB
	}
	j1 := joins[(i+1)%n]
	end := j1.m
	if j1.arc {
		end = j1.pA
	}
	return start, end
}

// capWallSweep returns a circular wall's own CAP-LEVEL angular window — the
// offset corner feet's angles about the wall's exact centre — unwrapped to
// the branch nearest the wall's own recorded sweep refSweep (w.th1 - w.th0).
// A chamfer setback is small relative to a sane wall radius, so the offset
// foot never turns the corner's point by anywhere near a half turn from the
// wall's own endpoint; picking the nearest branch is what keeps delta small
// in patchRawFlux's own ruled-angle term rather than off by a spurious full
// turn. wraps is how many extra full turns that unwrap added, so
// capWallArcBound's own exact bracket can reproduce the same branch.
func capWallSweep(cU, cV float64, start, end Point2, refSweep float64) (capTh0, capTh1 float64, wraps int) {
	capTh0 = math.Atan2(start.V-cV, start.U-cU)
	raw1 := math.Atan2(end.V-cV, end.U-cU)
	diff := raw1 - capTh0
	for diff-refSweep > math.Pi {
		diff -= 2 * math.Pi
		wraps--
	}
	for diff-refSweep < -math.Pi {
		diff += 2 * math.Pi
		wraps++
	}
	return capTh0, capTh0 + diff, wraps
}

// oneLoopCornerLoop decomposes a single recorded loop into its coalesced
// corner walk, the same decomposition prismCornerLoopsBudget applies to
// every loop of a section.
func oneLoopCornerLoop(budget *proofbound.WorkBudget, loop LoopRecord, work *freeform.FreeformWork) (cornerLoop, error) {
	raw := make([]survey2d.SideWalk, len(loop.Segments))
	for i, seg := range loop.Segments {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return cornerLoop{}, err
		}
		w, err := walkOf(seg, work)
		if err != nil {
			return cornerLoop{}, err
		}
		if err := requireAnalyticWalk(w, "a cap-loop chamfer"); err != nil {
			return cornerLoop{}, err
		}
		raw[i] = survey2d.SideWalk{SegmentWalk: w, Segs: []int{i}}
	}
	walks, err := coalesceWalksBudget(raw, budget)
	if err != nil {
		return cornerLoop{}, err
	}
	return cornerLoop{walks: walks}, nil
}

// capBandResult is what buildCapBand contributes to the enclosing build: the
// new patch faces, the cap-level coedges bounding the loop's rewritten cap
// boundary (in walk order), the exact-rational patch geometry the moments
// pass integrates, and the band's own contour displacement (delta,
// capblend_contour.go's capContourDelta/capWholeCircleDelta) — computed once
// here, since buildCapBand already needs it for every cap-level vertex and
// edge bound, and returned so the cap face area and band volume readings
// (capblend_moments.go) charge the SAME value rather than each re-deriving it
// through loopContourDelta.
type capBandResult struct {
	patches []*Face
	capCo   []coedge
	geom    []capPatchGeom
	delta   float64
}

type capPatchGeom = capband.Patch

// buildCapBand builds the chamfer band for one loop selected on one cap: the
// patch faces between the cap contour (offset dc into material, at capZ) and
// the side contour (the original loop, at capZ + matSign*ds), and the
// cap-level coedges that replace the loop's boundary in the cap face. The
// side-level boundary reuses the trimmed side wall's own near-cap coedges
// (sideCo, from buildLoopSidesAs) — shared, never re-derived.
func buildCapBand(ctx context.Context, body *Body, ref producerID, cbp capBlendPayload, li int, loop LoopRecord, capZ float64, matSign float64, sideCo []coedge, work *freeform.FreeformWork) (capBandResult, error) {
	if err := ctx.Err(); err != nil {
		return capBandResult{}, err
	}
	budget := proofbound.NewWorkBudget(ctx)
	cl, err := oneLoopCornerLoop(budget, loop, work)
	if err != nil {
		return capBandResult{}, err
	}
	walks := cl.walks
	n := len(walks)
	// dc offsets the cap contour in the plane and ds carries the original loop
	// along the sweep to the side level (docs/modify-reach-design.md §8.3.1).
	setback := cbp.setbackAt(matSign)
	dc, dcDelta, ds := setback.dc, setback.dcDelta, setback.ds
	sideZ := capZ + matSign*ds
	pl := cbp.prismLike(0, 0)

	capName := capNameOf(matSign)

	liftCap := func(p Point2) r3.Vec { return pl.point(p.U, p.V, capZ) }
	liftSide := func(p Point2) r3.Vec { return pl.point(p.U, p.V, sideZ) }

	// capVertexAt places one cap-level vertex at p, bounded by base beside the
	// vertex's own exact frame lift and placement rounding
	// (prismPayload.liftedVertex; topology.go's Vertex.Position contract) —
	// zero wherever that lift is exact, which is what keeps an ordinary
	// chamfer's cap-level vertices Exact. The lift term widens only the VERTEX
	// bound, never capLevelDelta itself, which the band's slant and cap edges
	// still read unwidened for their own, separate bound: the frame lift is a
	// per-coordinate rounding, not a chord or locus term, so it has no business
	// in an edge's length bound.
	capVertexAt := func(p Point2, base float64) *Vertex {
		held, lift := pl.liftedVertex(p.U, p.V, capZ)
		return &Vertex{position: held, bound: units.Millimeters(proofbound.AbsSumUpper(base, lift))}
	}

	// levelDelta is the side level's conversion and float-sum rounding: sideZ
	// is a float sum, so the band's side directrix sits that far from the level
	// it denotes, and every edge with an endpoint there carries it beside the
	// contour's own displacement. It is the same term capBandVolume charges for
	// the identical level, and it rides onto every patch's own capPatchGeom,
	// where patchAreaOf charges it against the patch's area
	// (capblend_moments.go).
	levelDelta := proofbound.AbsSumUpper(setback.dsDelta, proofarith.AddRoundError(capZ, matSign*ds, sideZ))
	// capDelta is the inherited displacement of the cap level itself. The cap
	// contour moves only in the cap plane, so its delta does not cover this
	// independent axial term.
	capDelta := cbp.capBandLevel(capZ, matSign).Bound

	// A single closed circle has no corner: one Cone patch, full turn.
	if n == 1 && walks[0].Closed {
		w := walks[0]
		capRadius, err := capBandRadius(w, dc)
		if err != nil {
			return capBandResult{}, err
		}
		delta, err := capWholeCircleDelta(w, dc, dcDelta)
		if err != nil {
			return capBandResult{}, err
		}
		exactRadius, ok := capOffsetRadiusSpan(w, dc, dcDelta)
		if !ok {
			return capBandResult{}, errCapContourUnbounded
		}
		seam0 := sideCo[0].edge // the side wall's own whole-circle bottom/top edge
		capLevelDelta := proofbound.AbsSumUpper(delta, capDelta)
		// wholeCircleEdge's own delta parameter feeds ONLY its seam vertex's
		// bound — its returned Edge's own lengthBound comes from
		// capCircleLengthBound instead — and the seam vertex adds its own
		// exact lift rounding there.
		capEdge := wholeCircleEdge(pl, w.CU, w.CV, capRadius, capZ, w.Th1 > w.Th0, capLevelDelta, exactRadius)
		patch := buildConePatch(pl, body, ref, li, 0, w.CU, w.CV, w.Radius, capRadius, sideZ, capZ, matSign, false, seam0, capEdge)
		sign := 1.0
		if w.Th1 < w.Th0 {
			sign = -1
		}
		samplePoint := pl.point(w.CU+w.Radius, w.CV, sideZ)
		if err := fixPatchOrientation(patch, pl, samplePoint, sign, 0, -matSign); err != nil {
			return capBandResult{}, err
		}
		// th0, th1 record the patch's ANGULAR EXTENT, not the wall's own
		// walked sense: the DX7 survey reads the window as an increasing
		// pair. The sense itself is not discarded — sweepCCW keeps it, and
		// patchRawFlux negates a clockwise-walked patch's flux with it.
		gth0, gth1 := w.Th0, w.Th1
		if gth1 < gth0 {
			gth0, gth1 = gth1, gth0
		}
		// chordUpper/slant are this whole-circle patch's own held chord (its
		// full circumference) and slant distance (the radial change ruled
		// against the axial one), the two factors proofbound.BandPatchAreaAllow needs.
		chordUpper := proofbound.AbsSumUpper(capEdge.length, capEdge.lengthBound)
		slant := math.Hypot(math.Abs(capRadius-w.Radius), math.Abs(capZ-sideZ))
		// CapThAllow: the whole-turn patch's cap-level sweep is the WALL's own
		// recorded th0/th1 (there is no offset corner to trim it — wholeTurn is
		// exactly this structural fact), so its true value is 2π as a fact of
		// the construction rather than something an offset solve computed. The
		// only inexactness is the wall's own float representation of a full
		// turn, bracketed against 2π's own exact rational constants — the same
		// bracket capCircleLengthBound already takes for this circle's
		// circumference.
		dthHeld := gth1 - gth0
		capThAllow := proofbound.IntervalFloatError(proofbound.TwoPiInterval(), dthHeld)
		geom := capPatchGeom{Circular: true, CU: w.CU, CV: w.CV, SideRadius: w.Radius, CapRadius: capRadius, Th0: gth0, Th1: gth1, CapTh0: gth0, CapTh1: gth1, SweepCCW: w.Th1 > w.Th0, WholeTurn: true, SideZ: sideZ, CapZ: capZ, ContourAllow: proofbound.BandPatchAreaAllow(delta, chordUpper, slant), LevelDelta: levelDelta, CapThAllow: capThAllow}
		// The two circles pair at their seams, the side wall's own walk start
		// and the cap circle's (cU + capRadius, cV), and keep that pairing all
		// the way round, so both corner skews are the seams' one angle.
		sideSeam := Point2{U: w.StartU, V: w.StartV}
		capSeam := Point2{U: w.CU + capRadius, V: w.CV}
		if err := setCapPatchSkews(&geom, sideSeam, sideSeam, capSeam, capSeam); err != nil {
			return capBandResult{}, err
		}
		// A cornerless band has no corner to pair its two directrices at, so
		// the one ruling its azimuth spread is measured on is the pair of SEAM
		// vertices — the one place either circle names a parameter origin.
		setPatchReadings(patch, geom, capBuiltPatch(seam0, capEdge, []*Vertex{seam0.Start()}, []*Vertex{capEdge.Start()}))
		capLoop := []coedge{{edge: capEdge, forward: true}}
		return capBandResult{patches: []*Face{patch}, capCo: capLoop, geom: []capPatchGeom{geom}, delta: delta}, nil
	}

	joins, err := capOffsetJoins(budget, cl, dc)
	if err != nil {
		return capBandResult{}, err
	}

	// delta is this band's CONTOUR DISPLACEMENT (capblend_contour.go): the one
	// proven upper bound on how far a cap-level point the build emits sits from
	// the point the offset denotes. Every cap-level vertex and every cap-level
	// edge below is bounded through it, so no two readers of the same contour
	// are told a different story about it — and neither the zero bound a
	// recorded coordinate earns nor an infinite one that bounds nothing is
	// published for a coordinate this solve computed.
	delta, err := capContourDelta(walks, joins, dc, dcDelta)
	if err != nil {
		return capBandResult{}, err
	}
	capLevelDelta := proofbound.AbsSumUpper(delta, capDelta)

	// sideVertexAt(i) is the ORIGINAL corner point before wall i, at sideZ —
	// buildLoopSidesAs's own shared vertex (sideCo[i].edge.Start() ==
	// sideCo[i-1].edge.End()), reused rather than re-created, so a reflex
	// corner's apex patch and its two neighboring wall patches close on the
	// SAME vertex.
	sideVertexAt := func(i int) *Vertex { return sideCo[i].edge.Start() }

	// Pass 1: every corner's connector edge(s), independent of wall build
	// order (so a reflex corner at index 0 wraps around correctly). A miter
	// corner (joins[i].m) has ONE edge shared by the wall before and the
	// wall after it: slantOut[i] == slantIn[i]. A reflex corner has TWO —
	// slantIn[i] (from the offset foot pA, used by the PRECEDING wall's
	// trailing edge) and slantOut[i] (from pB, used by the FOLLOWING wall's
	// leading edge) — different cap-level points, but both ending at the
	// SAME side-level apex vertex, sideVertexAt(i).
	slantIn := make([]*Edge, n)
	slantOut := make([]*Edge, n)
	// slantInHeld/slantOutHeld are slantIn[i]/slantOut[i]'s own ARITHMETIC-ONLY
	// length bound (capSlantEdge's second return) — the same value the edge's
	// own lengthBound carries everywhere except a miter ruling adjacent to a
	// circular wall, where lengthBound also carries the chord-versus-locus
	// excess (docs/modify-reach-design.md §8.3) that proofbound.BandPatchAreaAllow's
	// slantUpper must NOT read (task-file rule: the area term stays on the
	// held chord alone).
	slantInHeld := make([]float64, n)
	slantOutHeld := make([]float64, n)
	arcByCorner := make([]*Edge, n) // non-nil for a reflex corner's cap-level arc
	arcTh0 := make([]float64, n)
	arcTh1 := make([]float64, n)
	// arcWraps carries each reflex corner's own unwrap count (below) into
	// Pass 2's capThAllow computation, which needs the SAME branch this
	// pass's arcTh0[i]/arcTh1[i] already committed to — recomputing it from
	// those two alone would lose which multiple of 2π they were unwrapped by.
	arcWraps := make([]int, n)
	for i := range n {
		j := joins[i]
		apex := sideVertexAt(i)
		prev, cur := walks[(i+n-1)%n], walks[i]
		if !j.arc {
			capV := capVertexAt(j.m, capLevelDelta)
			e, held, err := capSlantEdge(budget, j.m, capV, apex, j.vU, j.vV, capZ, sideZ, capLevelDelta, levelDelta, prev, cur, setback, j.g1)
			if err != nil {
				return capBandResult{}, err
			}
			slantIn[i], slantOut[i] = e, e
			slantInHeld[i], slantOutHeld[i] = held, held
			continue
		}
		pAV := capVertexAt(j.pA, capLevelDelta)
		pBV := capVertexAt(j.pB, capLevelDelta)
		var errA, errB error
		slantIn[i], slantInHeld[i], errA = capSlantEdge(budget, j.pA, pAV, apex, j.vU, j.vV, capZ, sideZ, capLevelDelta, levelDelta, survey2d.SideWalk{}, survey2d.SideWalk{}, capSetback{}, true)
		if errA != nil {
			return capBandResult{}, errA
		}
		slantOut[i], slantOutHeld[i], errB = capSlantEdge(budget, j.pB, pBV, apex, j.vU, j.vV, capZ, sideZ, capLevelDelta, levelDelta, survey2d.SideWalk{}, survey2d.SideWalk{}, capSetback{}, true)
		if errB != nil {
			return capBandResult{}, errB
		}
		th0 := math.Atan2(j.pA.V-j.vV, j.pA.U-j.vU)
		th1 := math.Atan2(j.pB.V-j.vV, j.pB.U-j.vU)
		// The inward offset's reflex connector walks CLOCKWISE from pA to pB
		// (shell_offset.go's arcSegment with ccw = s < 0, and this band offsets
		// with s = +1), so th1 is normalized BELOW th0 and the swept angle is
		// the corner's own reflex turn — strictly less than a half turn — never
		// its 2*pi complement. Taking the complement records the arc that runs
		// the wrong way around the corner: it lands on a cap-level curve the
		// offset never emitted, and every reader of the window (the edge's own
		// length, the band's flux integral, the patch's normal range) inherits
		// that error.
		wraps := 0
		for th1 > th0 {
			th1 -= 2 * math.Pi
			wraps++
		}
		arcWraps[i] = wraps
		arcTh0[i], arcTh1[i] = th0, th1
		arcLength := dc * (th0 - th1)
		arcByCorner[i] = &Edge{
			curve: Arc3{Center: liftCap(Point2{U: j.vU, V: j.vV}), Axis: pl.dir(0, 0, 1).Scale(-1), Radius: units.Millimeters(dc)},
			start: pAV, end: pBV,
			convex: false,
			length: arcLength, lengthBound: capApexArcBound(j, dc, dcDelta, arcLength, wraps, delta),
		}
	}

	var patches []*Face
	var geoms []capPatchGeom
	capCo := make([]coedge, 0, 2*n)

	// Pass 2: the apex patches (Cone-with-corner-apex, one per reflex
	// corner), independent of wall order.
	for i := range n {
		if !joins[i].arc {
			continue
		}
		j := joins[i]
		arc := arcByCorner[i]
		// p is Table BX row BX3's "deterministic patch order in the result's
		// own capBlendPayload" — the patch's index in this band's own patch
		// slice, which is the order capblend_moments.go pairs its geometry in.
		// It is NEVER the corner or wall index: those two index spaces collide
		// (corner i sits at walks[i].start), and two faces sharing one role
		// string collapse every last-wins reader — facesByRole, FaceCreatedBy,
		// and the survey's own role lookup alike — onto whichever face was
		// built second.
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, li, len(patches))
		surf := coneSurface(pl, j.vU, j.vV, 0, dc, sideZ, capZ)
		// Walk order: arc (pAV -> pBV, cap level), slantOut forward
		// (pBV -> apex), slantIn reversed (apex -> pAV) — a closed
		// triangle-like boundary each coedge's end matching the next's start.
		face := &Face{
			surface: surf,
			origins: []FeatureRef{{producer: ref, Role: role}},
			body:    body,
			loops: []*Loop{{coedges: []coedge{
				{edge: arc, forward: true},
				{edge: slantOut[i], forward: true},
				{edge: slantIn[i], forward: false},
			}, outer: true}},
		}
		// Orientation: a corner's own offset carrier is an EROSION, and at a
		// REFLEX corner the eroded boundary is the arc of radius dc about the
		// corner with the surviving material radially OUTSIDE it — the removed
		// wedge is the disk sector the arc cuts off, so this patch's cone has
		// the VOID inside it and the solid outside. Its outward normal
		// (-ds·cos, -ds·sin, -matSign·dc) therefore points radially INWARD,
		// toward the corner's own axis, and toward the chamfered cap
		// (-matSign), exactly as a regular wall patch's does axially; its dot
		// product with the reference below is dc + ds > 0 for every setback
		// pair (docs/modify-reach-design.md §8.3.1). The radially-outward
		// reading is not merely the wrong sign: its dot product is dc - ds, so
		// at dc = ds (cos, sin, -matSign) is EXACTLY perpendicular to the true
		// normal and decides nothing at all. Verified empirically (never hand-trusted):
		// fixPatchOrientation checks the actual built surface's own NormalAt
		// against this reference and reverses only if they disagree.
		if err := fixPatchOrientation(face, pl, pl.point(j.vU+dc*math.Cos(arcTh0[i]), j.vV+dc*math.Sin(arcTh0[i]), capZ), -math.Cos(arcTh0[i]), -math.Sin(arcTh0[i]), -matSign); err != nil {
			return capBandResult{}, err
		}
		slantIn[i].faces = append(slantIn[i].faces, face)
		arc.faces = append(arc.faces, face)
		slantOut[i].faces = append(slantOut[i].faces, face)
		patches = append(patches, face)
		// th0, th1 record the patch's ANGULAR EXTENT, not the connector's own
		// clockwise walk — the same normalization every other patch's geometry
		// takes, since the DX7 survey reads an increasing window. The connector
		// arc is walked CLOCKWISE (arcTh1 below arcTh0 by construction), and
		// sweepCCW is what carries that fact to patchRawFlux.
		gth0, gth1 := arcTh0[i], arcTh1[i]
		if gth1 < gth0 {
			gth0, gth1 = gth1, gth0
		}
		// chordUpper/slant are this apex patch's own held chord (the arc
		// joining the two offset feet) and slant distance (either of its two
		// triangle-like edges from an offset foot down to the shared
		// side-level apex — the two are not necessarily equal, so the larger
		// is the sound upper bound).
		chordUpper := proofbound.AbsSumUpper(arc.length, arc.lengthBound)
		slant := math.Max(
			proofbound.AbsSumUpper(slantOut[i].length, slantOutHeld[i]),
			proofbound.AbsSumUpper(slantIn[i].length, slantInHeld[i]),
		)
		// CapThAllow: this apex patch's own connector runs pA -> pB (the same
		// capApexArcBound bracket capApexArcBound already builds for the
		// connector's own arc LENGTH above), so capSweepAllow reads the
		// identical proofbound.Atan2Interval enclosure and reports it against the raw
		// sweep gth1-gth0 = arcTh0[i]-arcTh1[i] instead of against d*sweep.
		// start/end are passed as (pB, pA) so the bracket's own end-minus-start
		// convention reproduces atan2(pA)-atan2(pB), matching arcWraps[i]'s own
		// unwrap direction (Pass 1's "for th1 > th0 { th1 -= 2*math.Pi }").
		capThAllow := capSweepAllow(j.vU, j.vV, dc, j.pB, j.pA, arcTh0[i]-arcTh1[i], arcWraps[i], delta)
		g := capPatchGeom{
			Circular: true, SweepCCW: false,
			CU: j.vU, CV: j.vV, SideRadius: 0, CapRadius: dc,
			Th0: gth0, Th1: gth1, CapTh0: gth0, CapTh1: gth1, SideZ: sideZ, CapZ: capZ,
			ContourAllow: proofbound.BandPatchAreaAllow(delta, chordUpper, slant),
			LevelDelta:   levelDelta,
			CapThAllow:   capThAllow,
		}
		// The apex patch's rulings all leave the ORIGINAL corner vertex, which
		// is the cone tag's own apex: a point side directrix.
		setPatchReadings(face, g, capBuiltApexPatch(sideVertexAt(i), arc, []*Vertex{arc.Start(), arc.End()}))
		geoms = append(geoms, g)
	}

	// Pass 3: the wall patches (Plane or Cone) AND the cap-level boundary
	// coedges, in walk order — a wall's own capEdge, then the reflex arc
	// (if any) at the corner it leads into, exactly the order offsetLoopBudget
	// emits the same offset loop's segments in.
	for i := range n {
		w := walks[i]
		start, end := capWallFoot(joins, i, n)
		nextI := (i + 1) % n
		capA, capB := slantOut[i].start, slantIn[nextI].start
		side := sideCo[i].edge
		leadSlant, trailSlant := slantOut[i], slantIn[nextI]

		// A circular wall's cap-level radius is resolved ONCE per wall and
		// shared by the wall's edge, its surface and its recorded geometry, so
		// no two of them can disagree about the offset and capBandRadius's own
		// refusals are decided before any of the three is built. Its cap-level
		// SWEEP (capTh0, capTh1) is resolved alongside it: the offset corner
		// feet's own angles about the wall's exact centre, generally different
		// from the wall's own w.th0/w.th1 wherever the corner is a genuine
		// (non-tangent) miter (docs/modify-reach-design.md §8.3) — the cap
		// directrix is TRIMMED there, the side directrix is not.
		capRadius := 0.0
		capTh0, capTh1, wraps := 0.0, 0.0, 0
		capThAllow := 0.0
		radialShift := 0.0
		if w.IsCircular() {
			r, err := capBandRadius(w, dc)
			if err != nil {
				return capBandResult{}, err
			}
			radius, ok := capOffsetRadiusSpan(w, dc, dcDelta)
			if !ok {
				return capBandResult{}, errCapContourUnbounded
			}
			capRadius = r
			// radialShift is how far every radius the cap arc denotes sits
			// from the held capRadius: the offset radius R ∓ dc rounds even
			// for a setback stated in millimetres, and dc's own conversion
			// rounding widens it further. capWallArcBound charges it once per
			// radian the arc sweeps.
			radialShift = proofbound.IntervalFloatError(radius, capRadius)
			capTh0, capTh1, wraps = capWallSweep(w.CU, w.CV, start, end, w.Th1-w.Th0)
			// capThAllow is capSweepAllow's own enclosure of THIS sweep
			// (capTh1-capTh0), computed here where start/end/wraps are still
			// the ones capWallSweep just resolved: patchAreaOf's later swap of
			// g.CapTh0/g.CapTh1 (below) negates the raw difference but not its
			// absolute value, and this bound is symmetric in sign (it bounds
			// |held-true|), so computing it once, pre-swap, stays valid after.
			capThAllow = capSweepAllow(w.CU, w.CV, capRadius, start, end, capTh1-capTh0, wraps, delta)
		}

		var capEdge *Edge
		if !w.IsCircular() {
			held := math.Hypot(end.U-start.U, end.V-start.V)
			// Both endpoints are contour points, so both carry the band's own
			// displacement; the square root's own committed error is measured
			// against the exact squared length.
			capEdge = &Edge{
				curve: Line3{}, start: capA, end: capB, convex: true,
				length:      held,
				lengthBound: capEdgeLengthBound(held, end, start, delta),
			}
		} else {
			sweepSigned := capTh1 - capTh0
			held := math.Abs(capRadius * sweepSigned)
			capEdge = arcEdge(pl, w.CU, w.CV, capRadius, capZ, capA, capB, capTh0, capTh1, held,
				capWallArcBound(w.CU, w.CV, start, end, capRadius, capRadius*sweepSigned, wraps, delta, radialShift))
		}

		var surf Surface
		if !w.IsCircular() {
			f, err := planeFromThree(liftSide(Point2{U: w.StartU, V: w.StartV}), liftSide(Point2{U: w.EndU, V: w.EndV}), capB.position)
			if err != nil {
				return capBandResult{}, err
			}
			surf = f
		} else {
			surf = coneSurface(pl, w.CU, w.CV, w.Radius, capRadius, sideZ, capZ)
		}

		// Walk order mirrors the ordinary prism side face's own convention
		// (extrude.go: bottom forward, right vertical forward, top reversed,
		// left vertical reversed): side forward (sideVertexAt(i) ->
		// sideVertexAt(nextI)), trailSlant reversed (its natural direction is
		// capB -> sideVertexAt(nextI), so reversed walks UP to capB),
		// capEdge reversed (capB -> capA), leadSlant forward (its natural
		// direction is capA -> sideVertexAt(i), walking back DOWN to close).
		// p is the patch's own index in this band's patch slice (Table BX row
		// BX3), so an apex patch minted in pass 2 and a wall patch minted here
		// never share a role — see the pass-2 comment.
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, li, len(patches))
		face := &Face{
			surface: surf,
			origins: []FeatureRef{{producer: ref, Role: role}},
			body:    body,
			loops: []*Loop{{coedges: []coedge{
				{edge: side, forward: true},
				{edge: trailSlant, forward: false},
				{edge: capEdge, forward: false},
				{edge: leadSlant, forward: true},
			}, outer: true}},
		}
		// Orientation: the reference is the ORIGINAL wall's own outward
		// convention (the same "tangent rotated a quarter turn" rule
		// extrude.go's prism side walls use — it already covers hole walls
		// through the wall's OWN walked sense, no separate case needed) plus
		// the toward-the-cap Z sense every patch in this band shares. Checked
		// empirically against the patch's own built NormalAt, never hand
		// trusted: fixPatchOrientation reverses only if they disagree.
		var refU, refV float64
		var samplePoint r3.Vec
		if !w.IsCircular() {
			refU, refV = w.TanInV, -w.TanInU
			samplePoint = liftSide(Point2{U: w.StartU, V: w.StartV})
		} else {
			sign := 1.0
			if w.Th1 < w.Th0 {
				sign = -1
			}
			refU, refV = sign*math.Cos(w.Th0), sign*math.Sin(w.Th0)
			samplePoint = pl.point(w.CU+w.Radius*math.Cos(w.Th0), w.CV+w.Radius*math.Sin(w.Th0), sideZ)
		}
		if err := fixPatchOrientation(face, pl, samplePoint, refU, refV, -matSign); err != nil {
			return capBandResult{}, err
		}
		side.faces = append(side.faces, face)
		trailSlant.faces = append(trailSlant.faces, face)
		capEdge.faces = append(capEdge.faces, face)
		leadSlant.faces = append(leadSlant.faces, face)
		patches = append(patches, face)

		g := capPatchGeom{SideZ: sideZ, CapZ: capZ, LevelDelta: levelDelta}
		if w.IsCircular() {
			g.Circular = true
			g.CU, g.CV = w.CU, w.CV
			g.SideRadius, g.CapRadius = w.Radius, capRadius
			// th0, th1 record the ANGULAR EXTENT, not the wall's own walked
			// sense — see the single-closed-circle branch's comment above;
			// the same normalization applies to a partial arc wall, and
			// sweepCCW keeps the sense the normalization drops.
			g.SweepCCW = w.Th1 > w.Th0
			g.Th0, g.Th1 = w.Th0, w.Th1
			g.CapTh0, g.CapTh1 = capTh0, capTh1
			g.CapThAllow = capThAllow
			side0, side1 := Point2{U: w.StartU, V: w.StartV}, Point2{U: w.EndU, V: w.EndV}
			cap0, cap1 := start, end
			if g.Th1 < g.Th0 {
				// The SAME swap, applied to both pairs together: g.Th0 must
				// keep pairing with g.CapTh0 (both the wall's OWN start
				// corner) and g.Th1 with g.CapTh1 (both its end corner), or
				// patchRawFlux's ruled-angle term pairs the wrong corners.
				g.Th0, g.Th1 = g.Th1, g.Th0
				g.CapTh0, g.CapTh1 = g.CapTh1, g.CapTh0
				side0, side1 = side1, side0
				cap0, cap1 = cap1, cap0
			}
			if err := setCapPatchSkews(&g, side0, side1, cap0, cap1); err != nil {
				return capBandResult{}, err
			}
		} else {
			g.SideA = Point2{U: w.StartU, V: w.StartV}
			g.SideB = Point2{U: w.EndU, V: w.EndV}
			g.CapA, g.CapB = start, end
		}
		// chordUpper/slant are this wall patch's own held chord (its cap-level
		// edge) and slant distance (either of its two bounding slant edges,
		// which can differ at a mitered corner — the larger is the sound
		// upper bound).
		chordUpper := proofbound.AbsSumUpper(capEdge.length, capEdge.lengthBound)
		slant := math.Max(
			proofbound.AbsSumUpper(leadSlant.length, slantOutHeld[i]),
			proofbound.AbsSumUpper(trailSlant.length, slantInHeld[nextI]),
		)
		g.ContourAllow = proofbound.BandPatchAreaAllow(delta, chordUpper, slant)
		// The two rulings this wall patch is bounded by, paired end for end:
		// leadSlant joins the side wall's own start vertex to capA, trailSlant
		// its end vertex to capB. Both patch kinds read the pair: a Plane
		// patch's tag is fixed through three of its four built corners, so the
		// fourth's own departure from it is what capPatchNormalAllow measures
		// there.
		setPatchReadings(face, g, capBuiltPatch(side, capEdge,
			[]*Vertex{side.Start(), side.End()}, []*Vertex{capA, capB}))
		geoms = append(geoms, g)
		capCo = append(capCo, coedge{edge: capEdge, forward: true})
		if arc := arcByCorner[nextI]; arc != nil {
			// shell_offset.go's reflex-corner arc walks pA -> pB in the
			// SAME sense as the boundary's own travel direction (inward,
			// s=+1: a clockwise arc whose tangent continues the walk).
			capCo = append(capCo, coedge{edge: arc, forward: true})
		}
	}
	return capBandResult{patches: patches, capCo: capCo, geom: geoms, delta: delta}, nil
}

// setCapPatchSkews stamps a circular wall patch's two corner skews: the proven
// angle between the side directrix's end side0 (side1) and the cap
// directrix's end cap0 (cap1) at the patch's th0 (th1) corner, on the branch
// the held windows name (capband.CornerSkewUpper). patchAreaOf's Cone arm
// reads them for the ruled patch's own distance from the frustum sector it
// publishes. A corner whose skew cannot be enclosed below a quarter turn is
// ErrUnsupported: the requested band exists, and only this evaluator cannot
// bound its area.
func setCapPatchSkews(g *capPatchGeom, side0, side1, cap0, cap1 Point2) error {
	c0, c1 := capband.WindowOnBranch(g.CapTh0, g.CapTh1, g.Th0)
	s0, ok0 := capband.CornerSkewUpper(g.CU, g.CV, side0, cap0, c0-g.Th0)
	s1, ok1 := capband.CornerSkewUpper(g.CU, g.CV, side1, cap1, c1-g.Th1)
	if !ok0 || !ok1 {
		return errCapPatchSkewUnbounded
	}
	g.SkewStart, g.SkewEnd = s0, s1
	return nil
}

// errCapPatchSkewUnbounded is the refusal for a circular band patch whose
// side and cap directrices turn a quarter turn or more apart at a corner,
// where patchAreaOf's proven area bound does not hold.
var errCapPatchSkewUnbounded = fmt.Errorf(`%w: a cap-loop chamfer's circular band patch turns its cap contour a quarter turn or more from its side wall at a corner, and this evaluator proves no area bound for that ruled patch`, ErrUnsupported)

// setPatchReadings gives a constructed chamfer patch the two readings its OWN
// geometry already states: the area and bound patchAreaOf gives
// (capblend_moments.go) — the same numbers the body's area sum is built from,
// so a caller reading `Face.Area()` and a caller reading `Body.Area()` are told
// the same thing about the same surface — and the bound its `Face.NormalAt`
// owes, which is how far the RULED surface the build assembles can depart from
// the surface this file tags it with (capblend_departure.go).
//
// built names that ruled surface through the numbers the body PUBLISHES for it:
// the two directrices' own circles and the rulings' own endpoints. It is passed
// rather than re-derived because the departure is a WORLD-space quantity — the
// placement rounds every one of those coordinates independently — and a
// plane-local description states none of it.
//
// Left unset, a patch Face reports a zero area with a zero bound, which
// `exactnessOf` publishes as an EXACT zero: not merely a missing reading but a
// positively wrong one, asserted as a fact about a face that plainly has area.
// A zero surface-departure term is the same kind of wrong answer one reading
// over: it omits a direction difference the built surface has, and the DX7
// undercut survey reads it. Face.NormalAt separately composes its own
// arithmetic bound, and the survey reads the value stored here rather than a
// second computation of it, so no two readers can be told different stories.
// Every patch this file builds passes through here, and each does so with the
// geometry the moments pass then integrates, so those two can never disagree
// either.
func setPatchReadings(f *Face, g capPatchGeom, built capPatchBuilt) {
	f.area, f.areaBound = capband.AreaOf(g)
	f.normalBound = capPatchNormalAllow(f, g, built)
}

// capSlantEdge is one cap-level contour point's slant edge down to the
// side-level apex it closes on. The apex is a recorded (u, v) at the band's own
// side level, so the edge's whole displacement is the contour's at one end and
// the level's rounding at the other — and the held length is the float square
// root's, charged what it committed against the exact rational squared length
// rather than an ulp contract math.Hypot does not offer.
//
// A MITER corner's ruling (affine false) is tagged Line3 but is the true
// denoted locus only where BOTH prev and cur are straight walls: the exact
// offset family's corner foot is otherwise a conic, and the chord this Edge
// holds understates its length (docs/modify-reach-design.md §8.3's boundary
// bullet). Where either wall is circular, dc's own offset range [0, dc]
// (capMiterLocusUpper widens it by dc's unit-conversion rounding dcDelta) is
// split into capMiterLocusSubdivisions sub-ranges, each enclosed through
// miterLocusSpeedUpper and turned into that sub-range's own locus-length
// upper bound via proofbound.ChordLocusLengthAllow (called with a zero chordUpper, which
// reduces it to the raw product); the sub-range bounds sum to the whole
// locus's own upper bound, and the excess over the held chord is charged
// once, at the end. Splitting matters most for miterLocusSpeedUpper's own
// circle-circle case, whose enclosure of the two carriers' own offset ranges
// is decorrelated (capcontour.CircleCircleLocusSpeedUpper's doc comment) and can
// inflate the published bound past the held chord itself on a sizeable
// setback over one wide range, where the narrower per-sub-range boxes stay
// tight enough not to; the line-circle case (capcontour.LineCircleLocusSpeedUpper) has
// no such looseness but still subdivides the same way, harmlessly. Where any
// sub-range's enclosure cannot be built, the edge refuses through
// lengthUnbounded rather than publish an understated bound. An AFFINE corner
// ruling (affine true) — a reflex corner's two feet, which ride one wall's own
// carrier, and a G1 join's foot v + s·dc·n̂ (modify §7) — is the denoted locus
// itself, so prev/cur/setback go unused and the term stays zero — the same zero a
// line-line miter gets.
//
// The second return is the edge's own ARITHMETIC-ONLY bound — heldBound,
// before any locus excess is folded in — which proofbound.BandPatchAreaAllow's
// slantUpper must keep reading (docs/modify-reach-design.md §8.4): the area
// term bounds the built quad against the ruled surface it denotes, a question
// the locus excess (a boundary-only reading) does not answer, so widening
// slantUpper by it would loosen the area bound for a residual it never
// measured.
//
// budget is the same counter buildCapBand already threads through this
// band's build; each sub-range's own enclosure charges one step, so a band
// with many mitered circular corners cannot spend unbounded work here any
// more than it can building the contour displacement itself.
func capSlantEdge(budget *proofbound.WorkBudget, capP Point2, capV, apex *Vertex, apexU, apexV, capZ, sideZ, delta, levelDelta float64, prev, cur survey2d.SideWalk, setback capSetback, affine bool) (*Edge, float64, error) {
	dc, dcDelta := setback.dc, setback.dcDelta
	held := math.Hypot(math.Hypot(capP.U-apexU, capP.V-apexV), capZ-sideZ)
	squared, squaredOK := dySquaredDistance3(capP.U, capP.V, capZ, apexU, apexV, sideZ)
	heldBound := straightEdgeBound(held, squared, squaredOK, delta, levelDelta)
	e := &Edge{
		curve: Line3{}, start: capV, end: apex, convex: true,
		length:      held,
		lengthBound: heldBound,
	}
	if affine || dc <= 0 || (!prev.IsCircular() && !cur.IsCircular()) {
		return e, heldBound, nil
	}
	chordUpper := proofbound.AbsSumUpper(held, heldBound)
	total, ok, err := capMiterLocusUpper(budget, prev, cur, apexU, apexV, setback.axialUpper(), dc, dcDelta)
	if err != nil {
		return nil, 0, err
	}
	if !ok {
		e.lengthBound = 0
		e.lengthUnbounded = true
		return e, heldBound, nil
	}
	if excess := total - chordUpper; excess > 0 {
		e.lengthBound = proofbound.AbsSumUpper(heldBound, proofbound.UpRound(excess))
	}
	return e, heldBound, nil
}

// capMiterLocusUpper is the proven upper bound on the LENGTH of the conic miter
// locus a mitered ruling stands for — the sum capSlantEdge charges its chord
// against, factored out so the tessellator can read the same number for its own
// locus-gap term (docs/tessellation-reach-design.md §7) instead of deriving a
// second bound from the same two carriers.
//
// The offset range [0, dc] — out to dc + dcDelta where the setback's unit
// conversion rounded by dcDelta — is split into capMiterLocusSubdivisions
// sub-ranges, each enclosed through miterLocusSpeedUpper and turned into that
// sub-range's own length upper bound by proofbound.ChordLocusLengthAllow (called
// with a zero chordUpper, which reduces it to the raw product); the sub-range
// bounds sum to the whole locus's own. axialUpper must be a proven upper bound
// on the denoted locus's axial rise, which is the side setback the caller
// stated (capSetback.axialUpper), not the held difference of the two float
// levels. ok is false where any sub-range's enclosure cannot be
// built, which every caller answers by withholding its reading rather than
// publishing an understated one. Each sub-range charges one budget step, so a
// band with many mitered circular corners cannot spend unbounded work here.
func capMiterLocusUpper(budget *proofbound.WorkBudget, prev, cur survey2d.SideWalk, apexU, apexV, axialUpper, dc, dcDelta float64) (float64, bool, error) {
	if dc <= 0 || !(axialUpper >= 0) || proofbound.IsNonFinite(axialUpper) {
		return 0, false, nil
	}
	// span is the largest offset amount the denoted locus reaches and rate the
	// smallest it can be divided by: dc itself for a setback stated in
	// millimetres, and dc widened by its unit conversion's rounding dcDelta
	// otherwise, so the sub-ranges cover the denoted locus's whole offset range
	// and its axial speed is read no slower than the denoted one.
	span, rate := dc, dc
	if dcDelta > 0 {
		span = proofbound.UpRound(dc + dcDelta)
		rate = freeform.DownRound(dc - dcDelta)
		if rate <= 0 {
			return 0, false, nil
		}
	}
	total := 0.0
	for _, r := range capMiterLocusRanges(span) {
		if err := survey2d.WallBudgetStep(budget); err != nil {
			return 0, false, err
		}
		t0, t1 := r[0], r[1]
		speed, ok := miterLocusSpeedUpper(prev, cur, t0, t1, apexU, apexV)
		if !ok {
			return 0, false, nil
		}
		// width bounds the sub-range's exact width from above, and axial
		// bounds the locus's axial rise over it, axialUpper·width/rate, from
		// above, so ChordLocusLengthAllow's axial speed axial/width is no
		// slower than the denoted locus's axialUpper/rate.
		width := proofbound.RatFloatUp(new(big.Rat).Sub(proofarith.FloatRat(t1), proofarith.FloatRat(t0)))
		axial := proofbound.DivUpper(proofbound.ProductUpper(axialUpper, width), rate)
		total = proofbound.AbsSumUpper(total, proofbound.ChordLocusLengthAllow(speed, width, axial, 0))
	}
	return total, true, nil
}

// capMiterLocusRanges splits the offset range [0, span] into
// capMiterLocusSubdivisions sub-ranges at the floats k·span/N. Each sub-range
// starts at the very float the previous one ends at, and the last ends at
// span, so the ranges cover [0, span] with no gap however k·step rounds. Two
// separately rounded ends, k·step + step for one range and (k+1)·step for the
// next, can leave a one-ulp offset range that no speed enclosure covers.
func capMiterLocusRanges(span float64) [capMiterLocusSubdivisions][2]float64 {
	var ranges [capMiterLocusSubdivisions][2]float64
	step := span / capMiterLocusSubdivisions
	t0 := 0.0
	for k := range capMiterLocusSubdivisions {
		t1 := float64(k+1) * step
		if k == capMiterLocusSubdivisions-1 {
			t1 = span
		}
		ranges[k] = [2]float64{t0, t1}
		t0 = t1
	}
	return ranges
}

// capMiterLocusSubdivisions is the fixed number of offset sub-ranges
// capSlantEdge splits a miter corner's own [0, dc] into (its doc comment
// above states why one range is too loose). A quarter disk of radius 10
// chamfered up to 4mm — approaching its own line/circle tangency at
// offset 5mm — stays comfortably under its own held chord at this count.
const capMiterLocusSubdivisions = 32

// wholeCircleEdge builds a full-circle Edge (Circle3) in the cap plane at z,
// the same seam-vertex convention extrude.go's singleClosed branch uses. delta
// is the contour displacement of the concentric offset circle and exactRadius
// encloses every radius it denotes, so the seam vertex publishes the displacement it
// actually has (its own coordinate is a float SUM of the centre and that
// radius, which rounds once more, and its own lift through pl's frame and
// placement rounds by what prismPayload.liftedVertex measures) and the
// circumference is bounded against π's own rational bracket.
func wholeCircleEdge(pl prismPayload, cu, cv, r, z float64, ccw bool, delta float64, exactRadius proofbound.RatInterval) *Edge {
	seamU := cu + r
	seamPos, lift := pl.liftedVertex(seamU, cv, z)
	seam := &Vertex{
		position: seamPos,
		bound:    units.Millimeters(proofbound.AbsSumUpper(proofbound.AbsSumUpper(delta, lift), proofarith.AddRoundError(cu, r, seamU))),
	}
	axis := pl.dir(0, 0, 1)
	if !ccw {
		axis = axis.Scale(-1)
	}
	held := 2 * math.Pi * r
	return &Edge{
		curve: Circle3{Center: pl.point(cu, cv, z), Axis: axis, Radius: units.Millimeters(r)},
		start: seam, end: seam,
		convex: ccw,
		length: held, lengthBound: capCircleLengthBound(exactRadius, held),
	}
}

// arcEdge builds an Arc3 Edge in the cap plane at z between the given
// vertices, walking the sense (th0, th1) the caller's own directrix does —
// the wall's own (th0, th1) for a whole-circle band, the cap-level
// (capTh0, capTh1) capWallSweep resolved for a regular wall's own trimmed
// arc. length is the held sweep r*|th1-th0| and lengthBound is
// capWallArcBound's (or capCircleLengthBound's) proven bound on it — the
// caller forms the two together so the bound is never computed against a
// value spelled a second time.
func arcEdge(pl prismPayload, cu, cv, r, z float64, start, end *Vertex, th0, th1, length, lengthBound float64) *Edge {
	axis := pl.dir(0, 0, 1)
	if th1 < th0 {
		axis = axis.Scale(-1)
	}
	return &Edge{
		curve: Arc3{Center: pl.point(cu, cv, z), Axis: axis, Radius: units.Millimeters(r)},
		start: start, end: end,
		convex: th1 > th0,
		length: length, lengthBound: lengthBound,
	}
}

// planeFromThree builds a Plane surface through three world points, its
// normal fixed by the winding p0->p1->p2 (right-hand rule); the caller flips
// the face's reversed bit if that does not turn out to be the outward sense.
func planeFromThree(p0, p1, p2 r3.Vec) (Plane, error) {
	u := p1.Sub(p0)
	v := p2.Sub(p0)
	f, err := r3.NewFrame(p0, u, v)
	if err != nil {
		return Plane{}, fmt.Errorf(`%w: a chamfer patch's three corners are degenerate: %s`, ErrDegenerate, err)
	}
	return Plane{Frame: f}, nil
}

// coneSurface builds the surface a circular chamfer patch's wall occupies from
// the two (z, r) pairs it is ruled between. The KIND is decided by what the two
// stored radii ARE, never by how close they are: two DIFFERENT radii name a
// cone, and a cone is what this returns, however small the difference —
// substituting a cylinder there discards the taper the surface exists to carry,
// and the DX7 undercut survey reads that taper directly off this surface's own
// NormalAt. A tolerance here would silently answer a whole scale of legitimate
// chamfers (a large radius with a small setback) with a shape of different
// geometry, which is a wrong answer rather than a coarse one.
//
// Cylinder is therefore reserved for radii that are EXACTLY equal — the one
// configuration in which the surface really is a cylinder. The cap-band callers
// never reach it: capBandRadius refuses an offset whose radial change rounded
// away before a patch is built from it, and an apex patch runs from radius 0 to
// the cap setback dc, which S13 already proved non-zero.
//
// For a cone, Origin is the apex where the ruling reaches radius 0 (which may
// lie outside [sideZ, capZ] for a regular frustum), Axis is the growth
// direction, and Radius/HalfAngle are read off the same two (z, r) pairs.
func coneSurface(pl prismPayload, cu, cv, r0, r1, z0, z1 float64) Surface {
	dz, dr := z1-z0, r1-r0
	if dr == 0 {
		return Cylinder{Origin: pl.point(cu, cv, z0), Axis: pl.dir(0, 0, 1), Radius: units.Millimeters(r0)}
	}
	apexZ := z0 - r0*dz/dr
	growth := 1.0
	if dz*dr < 0 {
		growth = -1
	}
	return Cone{
		Origin:    pl.point(cu, cv, apexZ),
		Axis:      pl.dir(0, 0, growth),
		Radius:    units.Millimeters(0),
		HalfAngle: units.Radians(math.Atan2(math.Abs(dr), math.Abs(dz))),
	}
}

// capPatchWindowSkew is one patch's own widest corner skew: a proven upper
// bound on the angle about the centre between the side directrix's end and
// the cap directrix's end at either corner (docs/modify-reach-design.md §8.3),
// the larger of the two skews setCapPatchSkews stamps from
// capband.CornerSkewUpper. It is never read off a difference of two float
// Atan2 angles, which can fall below the exact angle. It is zero for a reflex
// corner's apex patch, for a join whose two ends lie on one ray from the
// centre, and for a Plane patch, which reaches the denoted offset family
// exactly and is not ruled between two arcs at all.
//
// It is a question about the patch's own plane-local GEOMETRY — whether the
// build ruled a developable cone sector or a doubly-curved surface between two
// differently-swept arcs — and that is what DX8 asks of it (capBlendMinRadius),
// its one reader. The DX7 normal reading asks a different question: how far the
// BUILT surface points away from the tag, which the placement's own rounding
// moves whether or not the windows coincide. That one is measured in world
// space from the published numbers (capblend_departure.go), never from these
// two windows. The mesh reads it too, for the distance from the ruled patch to
// the cone it is tagged as (docs/tessellation-reach-design.md §7's skewGap).
func capPatchWindowSkew(g capPatchGeom) float64 {
	if !g.Circular {
		return 0
	}
	return math.Max(g.SkewStart, g.SkewEnd)
}

// buildConePatch builds a full-turn Cone chamfer patch (a whole circular
// loop's own chamfer) between two full-circle edges. reversed follows the
// original wall's own sense (a hole/clockwise wall's material lies outside
// its cylinder, so its chamfer cone's geometric normal needs reversing too —
// extrude.go's same rule for a clockwise circular wall).
func buildConePatch(pl prismPayload, body *Body, ref producerID, li, patchIdx int, cu, cv, sideRadius, capRadius, sideZ, capZ float64, matSign float64, reversed bool, sideEdge, capEdge *Edge) *Face {
	role := fmt.Sprintf("chamferCap(%s,%d,%d)", capNameOf(matSign), li, patchIdx)
	surf := coneSurface(pl, cu, cv, sideRadius, capRadius, sideZ, capZ)
	loops := []*Loop{
		{coedges: []coedge{{edge: sideEdge, forward: true}}, outer: true},
		{coedges: []coedge{{edge: capEdge, forward: false}}, outer: true},
	}
	face := &Face{surface: surf, origins: []FeatureRef{{producer: ref, Role: role}}, body: body, reversed: reversed, loops: loops}
	sideEdge.faces = append(sideEdge.faces, face)
	capEdge.faces = append(capEdge.faces, face)
	return face
}

const (
	capNameStart = "start"
	capNameEnd   = "end"
)

func capNameOf(matSign float64) string {
	if matSign < 0 {
		return capNameEnd
	}
	return capNameStart
}

// fixPatchOrientation empirically verifies a patch's outward normal sign
// against a plane-local reference direction (refU, refV, refZ), reversing
// face.reversed if the two disagree. samplePoint must lie on the patch's own
// built surface. It reads the surface's OWN Face.NormalAt — the exact
// formula every downstream reader (DX7's undercut survey included) uses — so
// no consumer can see a different answer than what was checked here, and the
// check never trusts a hand-derived sign convention: a Plane's u x v and a
// Cone's radially-outward formula both flip with which cap a band is on
// (docs/modify-reach-design.md §8.3), so this call is the ONE place that
// decides the sign, from the analytic geometry itself.
//
// When NormalAt itself refuses at samplePoint, the sign is a build-time
// question this evaluator cannot finish, so §11 gives it no undecided state:
// fixPatchOrientation returns SX15 (docs/modify-reach-design.md §4) as
// ErrUnsupported rather than leaving face.reversed at whatever the caller
// constructed it with. NormalAt's own refusal there is ErrDegenerate, but
// that sentinel and ErrUnsupported are opposite existence claims a caller
// branches on (capblend.go's wrapCapBlendAuditError), so the underlying error
// is folded in with %v, never %w.
func fixPatchOrientation(f *Face, pl prismPayload, samplePoint r3.Vec, refU, refV, refZ float64) error {
	n, err := f.NormalAt(samplePoint)
	if err != nil {
		return fmt.Errorf(`%w: a cap-loop chamfer band patch's outward orientation cannot be certified; its own normal reading refuses at the build's orientation sample point (%v)`, ErrUnsupported, err)
	}
	ref := pl.dir(refU, refV, refZ)
	if n.Value.Dot(ref) < 0 {
		f.reversed = !f.reversed
	}
	return nil
}
