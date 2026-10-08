package decad

import (
	"context"
	"errors"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file admits exact planar solids to the general pair relation of
// docs/multibody-dynamics-design.md §9.1 and caches each body's §9.2
// convexity certificate. internal/pair decides the relation over the exact
// snapshots built here; this file owns admission and the public report. A
// body whose held mesh is exact but stands for a true boundary up to a
// positive displacement δ is admitted too, and §10.4's band rule turns the
// held relation into the published one.

// planarHeldChord is the chord tolerance an all-planar held-mesh body is
// tessellated at. Nothing on an all-planar body is chorded, so the value only
// keys the body's tessellation cache.
const planarHeldChord = 1.0

// heldChordOf is the chord, in millimetres, a held mesh with a curved face is
// read at under req: the caller's HeldChord, zero when unset, which admits no
// curved body (docs/multibody-dynamics-design.md §10.4). Every reader of the
// chord goes through here, so the chord's source is chosen in one place.
func heldChordOf(req ContactRequest) float64 {
	return req.HeldChord.Base()
}

// planarConvexityEntry is a body's cached convexity certificate. It is read
// at the identity pose: every admitted pose is an affine map with a positive
// determinant, which preserves every orientation sign the certificate reads.
// It is keyed as the snapshot it was read off (planarSnapshotEntry).
type planarConvexityEntry struct {
	chordBits uint64
	chordFree bool
	convex    bool
}

// planarSnapshotEntry is a body's cached exact held snapshot at the identity
// query pose, the body's own placement applied, or its refusal (ok false).
// A snapshot that does not read the held chord (every exact family, and a
// held mesh whose every face is planar) is chordFree and serves every chord;
// a curved held mesh's serves only the chord whose bits it records. Its
// slices are shared by every reader and never written after it is stored.
// topology is the triangle set's combinatorial audit and derived data, which
// no pose changes (planar.PlanarTopology); a snapshot it refuses is a refusal.
type planarSnapshotEntry struct {
	chordBits uint64
	chordFree bool
	ok        bool
	solid     planar.PlanarSolid
	delta     proofarith.Dyadic
	topology  *planar.PlanarTopology
}

// servesChord reports whether the entry stands for the snapshot at chord.
func (e *planarSnapshotEntry) servesChord(bits uint64) bool {
	return e.chordFree || e.chordBits == bits
}

// classifyExactPlanarPair decides the pair through the exact planar kernel
// when both bodies are admitted. It reports false, leaving report untouched,
// when either body is not.
func classifyExactPlanarPair(ctx context.Context, report *ContactReport) (bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	chord := heldChordOf(report.Request)
	a, deltaA, okA, err := planarSolidAtPose(ctx, budget, report.A, report.PoseA, chord)
	if err != nil || !okA {
		return false, err
	}
	b, deltaB, okB, err := planarSolidAtPose(ctx, budget, report.B, report.PoseB, chord)
	if err != nil || !okB {
		return false, err
	}
	hint := planarHintOf(report.A, report.B)
	result, err := planar.ClassifyPlanarHinted(&a, &b, hint, budget.Step)
	if err != nil {
		return false, err
	}
	if result.Nearest != (planar.PlanarHint{}) && result.Nearest != hint {
		report.A.planarHints.Store(report.B, result.Nearest)
	}
	report.Relation, report.Gap, report.Overlap, report.Manifold = ContactUndecided, nil, nil, nil
	report.Reason = sourceBoxReason(result.Reason)
	if deltaA.Sign() > 0 || deltaB.Sign() > 0 {
		band := planarBandPair{a: &a, b: &b, deltaA: deltaA, deltaB: deltaB}
		if err := band.classify(ctx, budget, report, result); err != nil {
			return false, err
		}
		return true, budget.Err()
	}
	band := supportBandOf(report.Request)
	switch result.Relation {
	case pair.Separated:
		report.Relation = ContactSeparated
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
		if err := planarSupportBand(budget, report, &a, &b, result, band); err != nil {
			return false, err
		}
	case pair.Touching, pair.Overlapping:
		report.Relation = ContactOverlapping
		if result.Relation == pair.Touching {
			report.Relation = ContactTouching
			gap := Measurement{Value: units.Millimeters(0), Exactness: Exact, Bound: units.Millimeters(0)}
			report.Gap = &gap
		}
		// §9.3 and §9.6 need one convex side for a manifold; without one an
		// overlap's reason names the missing certificate, and a touch takes
		// §10.5's non-convex guest path inside publishPlanarManifold.
		convexA, err := planarConvexity(ctx, budget, report.A, chord)
		if err != nil {
			return false, err
		}
		convexB, err := planarConvexity(ctx, budget, report.B, chord)
		if err != nil {
			return false, err
		}
		if !convexA && !convexB && result.Relation == pair.Overlapping {
			report.Reason = ContactNonConvex
			break
		}
		report.Reason = ContactNoNormalProof
		if err := publishPlanarManifold(budget, report, &a, &b, result, convexA, convexB, band); err != nil {
			return false, err
		}
	default:
		if report.Reason == ContactNoReason {
			report.Reason = ContactAmbiguousFeature
		}
	}
	return true, budget.Err()
}

// planarHintOf returns the nearest candidate pair the last exact planar
// relation of a, as the first solid, against b found, or the zero hint. A
// hint never changes a relation (planar.ClassifyPlanarHinted), so one left by a
// pose far from this one, or by a snapshot at another chord, only costs time.
func planarHintOf(a, b *Body) planar.PlanarHint {
	stored, _ := a.planarHints.Load(b)
	hint, _ := stored.(planar.PlanarHint)
	return hint
}

// planarConvexity returns the body's cached §9.2 certificate, computing it at
// the identity pose on first use. A canceled computation caches nothing. A
// positive-displacement body's certificate is its held mesh's, read at chord
// when that mesh has a curved face.
func planarConvexity(ctx context.Context, budget *proofbound.WorkBudget, b *Body, chord float64) (bool, error) {
	bits := math.Float64bits(chord)
	if entry := b.planarConvexity.Load(); entry != nil && (entry.chordFree || entry.chordBits == bits) {
		return entry.convex, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.ok {
		return false, err
	}
	solid, ok, err := placePlanarSnapshot(budget, snapshot, r3.Identity())
	if err != nil || !ok {
		return false, err
	}
	convex, err := planar.PlanarConvex(&solid, budget.Step)
	if err != nil {
		return false, err
	}
	b.planarConvexity.Store(&planarConvexityEntry{chordBits: bits, chordFree: snapshot.chordFree, convex: convex})
	return convex, nil
}

// planarSolidAtPose builds the exact held boundary of an admitted planar
// solid under a query pose (§9): a prism whose section is all whole LineSeg
// edges with zero deltas, a zero-bound faceted Boolean, either directly or
// through its saved exact mesh and a translation-only placement, or a closed
// all-planar stitched solid welded at the identity with every vertex bound
// zero. Every vertex is the exact dyadic image of recorded coordinates under
// the recorded placement and the pose; nothing is read from a rounded
// transient body.
//
// §10.4 admits held meshes whose true boundary lies within a positive
// two-sided displacement δ of them: a positive-bound faceted Boolean, read
// off its payload with δ its mesh bound, a placed or certificate-welded
// stitched solid, read off its triangle set with δ its largest vertex bound,
// and every other solid payload without an exact contact family of its own,
// read off its VerifyAll tessellation with δ that mesh's Bound
// (planarHeldMeshSolid), a body with a curved face at chord. The
// returned displacement is δ at the query pose: the body-frame figure times
// an upper bound on the pose's stretch (planarPoseScale). It is zero for an
// exact body.
//
// The snapshot at the identity pose is built once per body and chord and
// cached on the body with its triangles' combinatorial audit
// (planarSnapshotOf); each call maps its vertices through the pose and audits
// the coordinate half (placePlanarSnapshot), so the cache changes no outcome.
func planarSolidAtPose(ctx context.Context, budget *proofbound.WorkBudget, b *Body,
	pose r3.Transform, chord float64) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if b == nil || !b.solid || b.kind != BodySolid || !positiveAffine(pose) {
		return planar.PlanarSolid{}, none, false, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.ok {
		return planar.PlanarSolid{}, none, false, err
	}
	solid, ok, err := placePlanarSnapshot(budget, snapshot, pose)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	delta := snapshot.delta
	if delta.Sign() > 0 {
		delta = proofarith.DyMul(delta, planarPoseScale(pose))
	}
	return solid, delta, true, nil
}

// placePlanarSnapshot maps a cached snapshot's vertices through pose into a
// fresh vertex slice and audits the result: the snapshot's topology already
// passed the combinatorial half of planar.CheckPlanarSolid, and
// planar.CheckPlanarPose runs the coordinate half and attaches the pose's
// derived data. The triangle and face slices are shared with the cache,
// clipped so no append can reach its storage.
func placePlanarSnapshot(budget *proofbound.WorkBudget, snapshot *planarSnapshotEntry,
	pose r3.Transform) (planar.PlanarSolid, bool, error) {
	solid := planar.PlanarSolid{Verts: make([]proofarith.DyV3, len(snapshot.solid.Verts)),
		Tris: slices.Clip(snapshot.solid.Tris), Faces: slices.Clip(snapshot.solid.Faces)}
	place := newExactContactMap(pose)
	for i, v := range snapshot.solid.Verts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		solid.Verts[i] = place.apply(v)
	}
	audited, err := planar.CheckPlanarPose(&solid, snapshot.topology, budget.Step)
	if err != nil || !audited {
		return planar.PlanarSolid{}, false, err
	}
	return solid, true, nil
}

// planarSnapshotOf returns the body's exact snapshot at the identity query
// pose for chord, building and caching it on a miss. An error (cancellation,
// an exhausted budget) caches nothing; a refusal is cached like a snapshot,
// since it reads nothing but the body and the chord.
func planarSnapshotOf(ctx context.Context, budget *proofbound.WorkBudget, b *Body, chord float64) (*planarSnapshotEntry, error) {
	bits := math.Float64bits(chord)
	if entry := b.planarSnapshot.Load(); entry != nil && entry.servesChord(bits) {
		return entry, nil
	}
	entry := &planarSnapshotEntry{chordBits: bits, chordFree: true, delta: proofarith.DyZero()}
	var err error
	switch payload := b.payload.(type) {
	case prismPayload:
		entry.solid, entry.ok, err = planarPrismSolid(ctx, budget, payload, prismFaceIndex(b))
	case facetedPayload:
		entry.solid, entry.delta, entry.ok, err = planarFacetedSolid(budget, payload)
	case stitchPayload:
		entry.solid, entry.delta, entry.ok, err = planarStitchSolid(budget, b, payload)
	case coilPayload:
		entry.solid, entry.delta, entry.ok, err = planarCoilSolid(ctx, budget, b, payload)
	default:
		entry.solid, entry.delta, entry.ok, entry.chordFree, err = planarHeldMeshSolid(ctx, budget, b, chord)
	}
	if err != nil {
		return nil, err
	}
	if entry.ok {
		entry.topology, entry.ok, err = planar.NewPlanarTopology(len(entry.solid.Verts), entry.solid.Tris, budget.Step)
		if err != nil {
			return nil, err
		}
	}
	if !entry.ok {
		entry.solid, entry.delta = planar.PlanarSolid{}, proofarith.DyZero()
	}
	b.planarSnapshot.Store(entry)
	return entry, nil
}

// planarPoseScale is an exact upper bound, at least one, on how far the
// linear part L of an admitted transform stretches any length. The largest
// absolute row sum g of the Gram matrix LᵀL bounds its largest eigenvalue,
// which is |L|², and (1 + g)/2 bounds √g from above. A valid r3 transform is
// orthonormal to rounding, so the bound sits a hair above one; it is charged
// all the same, since the held mesh and its displacement move through the
// exact float map, not through a rotation.
func planarPoseScale(t r3.Transform) proofarith.Dyadic { return planarsnapshot.PoseScale(t) }

func positiveAffine(t r3.Transform) bool { return planarsnapshot.PositiveAffine(t) }

// prismFaceIndex maps a prism face's provenance role to its index in the
// body's Faces order, the roles evalPrism gives its caps and side walls.
func prismFaceIndex(b *Body) map[string]int {
	index := make(map[string]int)
	for i, face := range b.Faces() {
		for _, origin := range face.Origins() {
			index[origin.Role] = i
		}
	}
	return index
}

// planarPrismSolid lifts an all-LineSeg section through its frame and sweep
// levels. The caps reuse the tessellator's triangulation, which this function
// then proves: every cap triangle is exactly counterclockwise and, through the
// closed-mesh audit, the caps' boundary chain is the section loops, so the
// triangles tile the section exactly. Each triangle records its face from
// the roles in faceIndex: the start cap at z0, the end cap at z1, and
// side(loop, segment) for a wall; a missing role leaves Faces unset.
func planarPrismSolid(ctx context.Context, budget *proofbound.WorkBudget,
	pp prismPayload, faceIndex map[string]int) (planar.PlanarSolid, bool, error) {
	return planarsnapshot.PrismSolid(ctx, budget, planarsnapshot.PrismInput{
		Outer: pp.profile.Outer, Holes: pp.profile.Holes,
		Frame: pp.frame, Transform: pp.xform, Z0: pp.z0, Z1: pp.z1,
		SectionDelta: pp.sectionDelta, Z0Delta: pp.z0Delta, Z1Delta: pp.z1Delta,
		SurfaceResult: pp.surfaceResult, CapStartRole: roleCapStart, CapEndRole: roleCapEnd,
	}, faceIndex, triangulation.Triangulate)
}

// planarFacetedSolid reads a Boolean's held mesh. A zero-bound mesh is its
// exact boundary. A translation-only placement reads the saved exact source
// mesh moved by the exact placement translation, after a reject-only check
// that every rebuilt vertex lies within the published bound of it. Any other
// positive-bound mesh is read as held, with its mesh bound returned as the
// displacement δ of §10.4.
func planarFacetedSolid(budget *proofbound.WorkBudget, pp facetedPayload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	return planarsnapshot.FacetedSolid(budget, planarsnapshot.FacetedInput{
		Verts: pp.verts, Tris: pp.tris, FaceOf: pp.faceOf,
		ExactSourceVerts: pp.exactSourceVerts, ExactSourceTris: pp.exactSourceTris,
		MeshBound: pp.meshBound, VolSymDiff: pp.volSymDiff, Transform: pp.xform,
	})
}

// planarHeldSolid lifts a held triangle mesh to an exact snapshot. faceOf
// names each triangle's face; a length mismatch leaves Faces unset.
func planarHeldSolid(budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int, faceOf []int) (planar.PlanarSolid, bool, error) {
	return planarsnapshot.HeldSolid(budget, verts, tris, faceOf)
}

// planarStitchSolid reads a closed all-planar stitched solid off its own
// audited triangle set (§9). Stitch records tris, triFaces and vertBound only
// for an all-planar weld, and auditClean only when the crossing audit ran and
// passed, so a curved or mixed stitch (nil tris) and a failed audit are not
// admitted; planarSolidAtPose has already refused an open sheet, which is
// not a solid. The vertices are the welded table's floats at the stitch's
// placement, triFaces names each triangle's live face, and a face missing
// from b.Faces() leaves the snapshot's face map unset.
//
// The displacement is the largest per-vertex weld bound, and at least the
// placement rounding delta. Each vertBound entry is the bound vertexForClass
// stamps on its live Vertex, the weld class bound with delta already added,
// and every held triangle point is a convex combination of its corners, so
// no true boundary point lies farther than that bound from the held mesh. It
// is zero for a weld built at the identity whose every class is zero-bound,
// which is then an exact §9 body.
func planarStitchSolid(budget *proofbound.WorkBudget, b *Body, sp stitchPayload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !sp.auditClean || len(sp.tris) == 0 || len(sp.triFaces) != len(sp.tris) ||
		len(sp.vertBound) != len(sp.verts) || !finiteMeasurementValues(sp.delta) || sp.delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	displacement := sp.delta
	for _, bound := range sp.vertBound {
		if !finiteMeasurementValues(bound) || bound < 0 {
			return planar.PlanarSolid{}, none, false, nil
		}
		displacement = max(displacement, bound)
	}
	faces := b.Faces()
	faceAt := make(map[*Face]int, len(faces))
	for i, face := range faces {
		faceAt[face] = i
	}
	faceOf := make([]int, len(sp.tris))
	for t, face := range sp.triFaces {
		at, ok := faceAt[face]
		if !ok {
			faceOf = nil
			break
		}
		faceOf[t] = at
	}
	solid, ok, err := planarHeldSolid(budget, sp.verts, sp.tris, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(displacement), true, nil
}

// planarCoilSolid reads a coil off its held shell (docs/helix-design.md Table
// CD row CD5). The shell is a closed all-triangle restatement of the body
// whose every vertex lies within its own β of the true boundary, so its
// displacement is the payload's delta, the same bound its mesh publishes. The
// restatement does not depend on a chord, so tessellateCoil is asked at delta
// itself, the finest tolerance it serves, and the snapshot serves every
// chord. The wall's Faceted face is not planar, which is why this path does
// not go through planarHeldMeshSolid: that arm refuses a curved body at the
// zero chord the planar arm reads.
func planarCoilSolid(ctx context.Context, budget *proofbound.WorkBudget, b *Body,
	cp coilPayload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !finiteMeasurementValues(cp.delta) || cp.delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	mesh, err := tessellateCoil(ctx, b, cp, cp.delta, VerifyNone)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, none, false, ctxErr
		}
		return planar.PlanarSolid{}, none, false, nil
	}
	faces := b.Faces()
	faceAt := make(map[*Face]int, len(faces))
	for i, face := range faces {
		faceAt[face] = i
	}
	faceOf := make([]int, len(mesh.triangles))
	for t, face := range mesh.source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarHeldSolid(budget, mesh.vertices, mesh.triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(cp.delta), true, nil
}

// planarHeldMeshSolid reads a solid payload without an exact contact family
// of its own (a cap-loop chamfer, a cup, a loft, a sweep, a general revolve)
// off its VerifyAll tessellation, the body's own held mesh at its placement,
// and returns that mesh's Bound as its displacement δ, the two-sided
// displacement between the mesh and the true boundary
// (docs/multibody-dynamics-design.md §10.4). A body whose every face is
// planar is read at planarHeldChord, which chords nothing, and its snapshot
// serves every chord (chordFree); a body with a curved face is read at chord,
// and a zero chord admits none. A source sphere or cylinder keeps its exact
// path and is not admitted here, nor is a body the tessellator refuses or
// whose mesh names a face the body does not carry.
func planarHeldMeshSolid(ctx context.Context, budget *proofbound.WorkBudget, b *Body,
	chord float64) (planar.PlanarSolid, proofarith.Dyadic, bool, bool, error) {
	none := proofarith.DyZero()
	if _, ok := sourceSphereRecord(b); ok {
		return planar.PlanarSolid{}, none, false, true, nil
	}
	if _, ok := sourceCylinderAtPose(b, r3.Identity()); ok {
		return planar.PlanarSolid{}, none, false, true, nil
	}
	faces := b.Faces()
	faceAt := make(map[*Face]int, len(faces))
	tolerance, chordFree := planarHeldChord, true
	for i, face := range faces {
		if !face.isPlanar() {
			tolerance, chordFree = chord, false
		}
		faceAt[face] = i
	}
	if !chordFree && tolerance <= 0 {
		return planar.PlanarSolid{}, none, false, false, nil
	}
	mesh, err := tessellateContext(ctx, b, units.Millimeters(tolerance), VerifyAll)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, none, false, chordFree, ctxErr
		}
		if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
			return planar.PlanarSolid{}, none, false, chordFree, err
		}
		return planar.PlanarSolid{}, none, false, chordFree, nil
	}
	if !mesh.boundaryOK || !finiteMeasurementValues(mesh.bound) || mesh.bound < 0 ||
		len(mesh.source) != len(mesh.triangles) {
		return planar.PlanarSolid{}, none, false, chordFree, nil
	}
	faceOf := make([]int, len(mesh.triangles))
	for t, face := range mesh.source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, chordFree, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarHeldSolid(budget, mesh.vertices, mesh.triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, chordFree, err
	}
	return solid, proofarith.MustDyOf(mesh.bound), true, chordFree, nil
}

// planarBandPair is one admitted pair at least one of whose held meshes
// carries a positive displacement, with both displacements at the query
// poses (docs/multibody-dynamics-design.md §10.4).
type planarBandPair struct {
	a, b           *planar.PlanarSolid
	deltaA, deltaB proofarith.Dyadic
}

// classify turns the exact held relation into §10.4's published one. With δ
// the summed displacement, every true boundary point lies within δ of the
// held pair's: a held gap within the lifted band b = max(SupportBand, δ) over
// which the zero-δ body hosts a lifted set is a band carrying that set
// (liftedBand); a held gap whose lower end exceeds δ is a true gap with δ
// charged; a held touch, or a held gap or penetration depth at most δ, puts
// the true signed separation in [−2δ, 2δ], ContactBand; a held vertex deeper
// than δ inside the other held body is a true overlap (planar.PlanarDeepVertex)
// carrying the held penetration patch charged with δ (overlapManifold).
// Anything between is undecided.
func (p *planarBandPair) classify(ctx context.Context, budget *proofbound.WorkBudget, report *ContactReport,
	result planar.PlanarResult) error {
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	switch result.Relation {
	case pair.Separated:
		value, bound := proofarith.FloatRat(result.Gap.ValueMM), proofarith.FloatRat(result.Gap.BoundMM)
		upper := new(big.Rat).Add(value, bound)
		if banded, err := p.liftedBand(budget, report, result, upper); err != nil || banded {
			return err
		}
		charged := new(big.Rat).Add(bound, delta)
		if published := proofbound.RatFloatUp(charged); finiteMeasurementValues(published) &&
			value.Cmp(proofarith.FloatRat(published)) > 0 {
			report.Relation, report.Reason = ContactSeparated, ContactNoReason
			report.Gap = &Measurement{Value: units.Millimeters(result.Gap.ValueMM),
				Bound: units.Millimeters(published), Exactness: exactnessFromBound(published)}
			return nil
		}
		if upper.Cmp(delta) <= 0 {
			return p.publishBand(ctx, budget, report, nil)
		}
		report.Reason = ContactNoGapProof
	case pair.Touching:
		convexA, convexB, err := p.convexity(ctx, budget, report)
		if err != nil {
			return err
		}
		if !convexA && !convexB {
			if err := p.publishBand(ctx, budget, report, nil); err != nil {
				return err
			}
			report.Reason = ContactNonConvex
			return nil
		}
		manifold, err := planar.PlanarTouchManifold(p.a, p.b, result.Contacts, convexA, convexB, budget.Step)
		if err != nil {
			return err
		}
		if err := p.publishBand(ctx, budget, report, manifold.Points); err != nil {
			return err
		}
		if manifold.Points == nil {
			report.Reason = sourceBoxReason(manifold.Reason)
		}
	case pair.Overlapping:
		deep, err := planar.PlanarDeepVertex(p.a, p.b, proofarith.DyAdd(p.deltaA, p.deltaB), budget.Step)
		if err != nil {
			return err
		}
		if deep {
			report.Relation, report.Reason = ContactOverlapping, ContactNoNormalProof
			return p.overlapManifold(ctx, budget, report, result)
		}
		report.Reason = ContactNoGapProof
		convexA, convexB, err := p.convexity(ctx, budget, report)
		if err != nil || !convexA || !convexB {
			return err
		}
		points, err := planar.PlanarPenetrationManifold(p.a, p.b, budget.Step)
		if err != nil || points == nil {
			return err
		}
		for _, point := range points {
			// The held depth is minus the separation's lower end.
			depth := new(big.Rat).Sub(proofarith.FloatRat(point.Separation.BoundMM),
				proofarith.FloatRat(point.Separation.ValueMM))
			if depth.Cmp(delta) > 0 {
				return nil
			}
		}
		return p.publishBand(ctx, budget, report, points)
	}
	return nil
}

// liftedBand publishes a held separated pair whose gap's upper end is within
// the lifted band b = max(SupportBand, δ) as ContactBand when some face of a
// zero-δ body hosts a nonempty lifted set within b (§10.4, §10.5). A support
// plane hosted by a displaced body publishes no lifted set: its held face
// only approximates the true face's direction. The band reads δ from below,
// so a resting displaced pair, placed with its held gap at most δ, keeps its
// lifted set however small SupportBand is. The gap becomes [0 ± (g + δ)], g
// the held gap's upper float, and the manifold is the lifted sets charged
// with δ (chargedManifold). It reports whether it published.
func (p *planarBandPair) liftedBand(budget *proofbound.WorkBudget, report *ContactReport, result planar.PlanarResult,
	upper *big.Rat) (bool, error) {
	band := p.liftedWidth(report.Request)
	hostA, hostB := p.deltaA.Sign() == 0, p.deltaB.Sign() == 0
	if upper.Cmp(band.Rat()) > 0 || !hostA && !hostB {
		return false, nil
	}
	lifted, err := planarLiftedSet(budget, p.a, p.b, result, planarSupportPlanes(p.a, p.b, hostA, hostB), band,
		false)
	if err != nil || len(lifted) == 0 {
		return false, err
	}
	g := proofbound.RatFloatUp(upper)
	charged := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(g), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !finiteMeasurementValues(g, charged) {
		return false, nil
	}
	report.Relation, report.Reason = ContactBand, ContactNoReason
	report.Gap = &Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(charged),
		Exactness: exactnessFromBound(charged)}
	manifold, reason, err := p.chargedManifold(budget, report, lifted, nil)
	if err != nil {
		return false, err
	}
	report.Manifold, report.Reason = manifold, reason
	return true, nil
}

// liftedWidth is the lifted band b = max(SupportBand, δ) of a displaced pair.
func (p *planarBandPair) liftedWidth(req ContactRequest) proofarith.Dyadic {
	band, delta := supportBandOf(req), proofarith.DyAdd(p.deltaA, p.deltaB)
	if proofarith.DyCmp(band, delta) < 0 {
		return delta
	}
	return band
}

// overlapManifold publishes the held penetration patch of a pair proven to
// overlap (§10.4): §9.3's convex-convex patch or §9.6's face-local one, each
// body read as M grown by its own δ, then the support plane's lifted set
// within the lifted band, every point charged with δ (chargedManifold). A
// patch the kernel withholds keeps the reason it names, or the one report
// carries.
func (p *planarBandPair) overlapManifold(ctx context.Context, budget *proofbound.WorkBudget, report *ContactReport,
	result planar.PlanarResult) error {
	convexA, convexB, err := p.convexity(ctx, budget, report)
	if err != nil {
		return err
	}
	if !convexA && !convexB {
		report.Reason = ContactNonConvex
		return nil
	}
	points, planes, reason, err := planarOverlapPatch(budget, p.a, p.b, result, convexA, convexB, p.deltaA, p.deltaB)
	if err != nil {
		return err
	}
	if points == nil {
		if reason != pair.NoReason {
			report.Reason = sourceBoxReason(reason)
		}
		return nil
	}
	manifold, contactReason, err := p.chargedManifold(budget, report, points, nil)
	if err != nil || manifold == nil {
		report.Reason = contactReason
		return err
	}
	lifted, err := planarLiftedSet(budget, p.a, p.b, result, planes, p.liftedWidth(report.Request), true)
	if err != nil {
		return err
	}
	if lifted != nil {
		extra, contactReason, err := p.chargedManifold(budget, report, lifted, nil)
		if err != nil || extra == nil {
			report.Reason = contactReason
			return err
		}
		manifold.Points = append(manifold.Points, extra.Points...)
	}
	report.Manifold, report.Reason = manifold, ContactNoReason
	return nil
}

func (p *planarBandPair) convexity(ctx context.Context, budget *proofbound.WorkBudget,
	report *ContactReport) (bool, bool, error) {
	chord := heldChordOf(report.Request)
	convexA, err := planarConvexity(ctx, budget, report.A, chord)
	if err != nil {
		return false, false, err
	}
	convexB, err := planarConvexity(ctx, budget, report.B, chord)
	return convexA, convexB, err
}

// publishBand publishes ContactBand with Gap [−2δ, 2δ] and, when points is
// not nil, the held manifold charged with the band, every Separation the
// band itself (chargedManifold).
func (p *planarBandPair) publishBand(ctx context.Context, budget *proofbound.WorkBudget, report *ContactReport,
	points []planar.PatchPoint) error {
	band := proofbound.RatFloatUp(new(big.Rat).Mul(big.NewRat(2, 1), proofarith.DyAdd(p.deltaA, p.deltaB).Rat()))
	if !finiteMeasurementValues(band) {
		report.Reason = ContactNoGapProof
		return nil
	}
	report.Relation, report.Reason = ContactBand, ContactNoNormalProof
	gap := Measurement{Value: units.Millimeters(0), Bound: units.Millimeters(band), Exactness: exactnessFromBound(band)}
	report.Gap = &gap
	if points == nil {
		return nil
	}
	if err := ctx.Err(); err != nil {
		return err
	}
	manifold, reason, err := p.chargedManifold(budget, report, points, &gap)
	if err != nil {
		return err
	}
	report.Manifold, report.Reason = manifold, reason
	return nil
}

// chargedManifold charges a held manifold with the displacements, as §9.4 and
// §10.4 state for a positive-displacement body: the points are the held
// pair's, each witness ball grows by its own body's δ, since a held vertex
// or foot is a point of the held mesh and Bound does not place it on the true
// surface, and each Separation is band when band is not nil and otherwise the
// held one widened by the summed δ. The normal must be the exact face normal
// of a body with no displacement, read at a face of that body that holds the
// point: a held face of a displaced body only approximates the true face's
// direction, so a point whose normal no exact face supplies withholds the
// manifold with ContactNoNormalProof.
func (p *planarBandPair) chargedManifold(budget *proofbound.WorkBudget, report *ContactReport, points []planar.PatchPoint,
	band *Measurement) (*ContactManifold, ContactReason, error) {
	exact := -1
	switch {
	case p.deltaA.Sign() == 0:
		exact = 0
	case p.deltaB.Sign() == 0:
		exact = 1
	}
	if exact < 0 {
		return nil, ContactNoNormalProof, nil
	}
	solids := [2]*planar.PlanarSolid{p.a, p.b}
	for _, point := range points {
		if err := budget.Step(); err != nil {
			return nil, ContactNoReason, err
		}
		feature := point.A
		if exact == 1 {
			feature = point.B
		}
		normal, ok := planarFaceNormal(solids[exact], feature)
		if !ok || !proofarith.DvIsZero(proofarith.DvCross(normal, point.Normal)) {
			return nil, ContactNoNormalProof, nil
		}
	}
	features, err := newPlanarFeatureMap(report.A, report.B, p.a, p.b)
	if err != nil {
		return nil, ContactNoReason, err
	}
	manifold, reason := planarPatchManifold(report.Request, points, features)
	if manifold == nil {
		return nil, reason, nil
	}
	resolution := proofarith.FloatRat(report.Request.PointResolution.Base())
	delta := proofarith.DyAdd(p.deltaA, p.deltaB).Rat()
	for i := range manifold.Points {
		point := &manifold.Points[i]
		for side, position := range []*VecMeasurement{&point.OnA, &point.OnB} {
			own := p.deltaA
			if side == 1 {
				own = p.deltaB
			}
			published := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(position.Bound.Base()), own.Rat()))
			if !finiteMeasurementValues(published) || proofarith.FloatRat(published).Cmp(resolution) > 0 {
				return nil, ContactPointTooCoarse, nil
			}
			position.Bound, position.Exactness = units.Millimeters(published), exactnessFromBound(published)
		}
		if band != nil {
			point.Separation = *band
			continue
		}
		widened := proofbound.RatFloatUp(new(big.Rat).Add(proofarith.FloatRat(point.Separation.Bound.Base()), delta))
		if !finiteMeasurementValues(widened) {
			return nil, ContactNoGapProof, nil
		}
		point.Separation.Bound, point.Separation.Exactness = units.Millimeters(widened), exactnessFromBound(widened)
	}
	return manifold, ContactNoReason, nil
}

// planarFaceNormal is the exact outward normal of the one face a facet
// feature names, read off the first triangle that face owns.
func planarFaceNormal(solid *planar.PlanarSolid, feature planar.PatchFeature) (proofarith.DyV3, bool) {
	if feature.Kind != planar.FeatureFacet || len(feature.Faces) != 1 || len(solid.Faces) != len(solid.Tris) {
		return proofarith.DyV3{}, false
	}
	for t, tri := range solid.Tris {
		if solid.Faces[t] != feature.Faces[0] {
			continue
		}
		a := solid.Verts[tri[0]]
		return proofarith.DvCross(proofarith.DvSub(solid.Verts[tri[1]], a), proofarith.DvSub(solid.Verts[tri[2]], a)), true
	}
	return proofarith.DyV3{}, false
}
