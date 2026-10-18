package decad

import (
	"context"
	"errors"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/measurement"
	"github.com/lestrrat-3d/decad/internal/pair/planar"
	"github.com/lestrrat-3d/decad/internal/planarband"
	"github.com/lestrrat-3d/decad/internal/planarsnapshot"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/reportvocab"
	"github.com/lestrrat-3d/decad/internal/selectorquery"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/triangulation"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file admits exact planar solids to the general pair relation of
// docs/multibody-dynamics-design.md §9.1 and caches each body's §9.2
// convexity certificate. internal/pair decides the relation over the exact
// snapshots built here; this file owns admission and the contact report. A
// body whose held mesh is exact but stands for a true boundary up to a
// positive displacement δ is admitted too; internal/planarband applies
// §10.4's band rule to the held relation.

// planarHeldChord is the chord tolerance an all-planar held-mesh body is
// tessellated at. Nothing on an all-planar body is chorded, so the value only
// keys the body's tessellation cache.
const planarHeldChord = 1.0

// heldChordOf is the chord, in millimetres, a held mesh with a curved face is
// read at under req: the caller's HeldChord, zero when unset, which admits no
// curved body (docs/multibody-dynamics-design.md §10.4). Every reader of the
// chord goes through here, so the chord's source is chosen in one place.
func heldChordOf(req reportvocab.ContactRequest) float64 {
	return req.HeldChord.Base()
}

// classifyExactPlanarPair decides the pair through the exact planar kernel
// when both bodies are admitted. It reports false, leaving report untouched,
// when either body is not.
func classifyExactPlanarPair(ctx context.Context, report *reportvocab.ContactReport) (bool, error) {
	budget := proofbound.NewWorkBudget(ctx)
	if err := budget.Err(); err != nil {
		return false, err
	}
	chord := heldChordOf(report.Request)
	a, deltaA, okA, err := planarSolidAtPose(ctx, budget, bodyHandle(report.A), report.PoseA, chord)
	if err != nil || !okA {
		return false, err
	}
	b, deltaB, okB, err := planarSolidAtPose(ctx, budget, bodyHandle(report.B), report.PoseB, chord)
	if err != nil || !okB {
		return false, err
	}
	hint := planarHintOf(report.A, report.B)
	result, err := planar.ClassifyPlanarHinted(&a, &b, hint, budget.Step)
	if err != nil {
		return false, err
	}
	if result.Nearest != (planar.PlanarHint{}) && result.Nearest != hint {
		report.A.PlanarHints.Store(report.B, result.Nearest)
	}
	report.Relation, report.Gap, report.Overlap, report.Manifold = reportvocab.ContactUndecided, nil, nil, nil
	report.Reason = sourceBoxReason(result.Reason)
	if deltaA.Sign() > 0 || deltaB.Sign() > 0 {
		services := planarband.Services{
			Convexity: func(ctx context.Context, budget *proofbound.WorkBudget, body *topo.Body,
				req reportvocab.ContactRequest) (bool, error) {
				return planarConvexity(ctx, budget, bodyHandle(body), heldChordOf(req))
			},
			FeatureMap: func(a, b *topo.Body, sa, sb *planar.PlanarSolid) (*reportvocab.PlanarFeatureMap, error) {
				return reportvocab.NewPlanarFeatureMap(bodyNode(bodyHandle(a)), bodyNode(bodyHandle(b)), sa, sb)
			},
			Patch:       planarPatchManifold,
			Reason:      sourceBoxReason,
			SupportBand: supportBandOf,
		}
		if err := planarband.Classify(ctx, budget, report, result, &a, &b, deltaA, deltaB, services); err != nil {
			return false, err
		}
		return true, budget.Err()
	}
	band := supportBandOf(report.Request)
	switch result.Relation {
	case pair.Separated:
		report.Relation = reportvocab.ContactSeparated
		gap := sourceBoxScalar(*result.Gap)
		report.Gap = &gap
		if err := planarSupportBand(budget, report, &a, &b, result, band); err != nil {
			return false, err
		}
	case pair.Touching, pair.Overlapping:
		report.Relation = reportvocab.ContactOverlapping
		if result.Relation == pair.Touching {
			report.Relation = reportvocab.ContactTouching
			gap := measurement.Measurement{Value: units.Millimeters(0), Exactness: measurement.Exact, Bound: units.Millimeters(0)}
			report.Gap = &gap
		}
		// §9.3 and §9.6 need one convex side for a manifold; without one an
		// overlap's reason names the missing certificate, and a touch takes
		// §10.5's non-convex guest path inside publishPlanarManifold.
		convexA, err := planarConvexity(ctx, budget, bodyHandle(report.A), chord)
		if err != nil {
			return false, err
		}
		convexB, err := planarConvexity(ctx, budget, bodyHandle(report.B), chord)
		if err != nil {
			return false, err
		}
		if !convexA && !convexB && result.Relation == pair.Overlapping {
			report.Reason = reportvocab.ContactNonConvex
			break
		}
		report.Reason = reportvocab.ContactNoNormalProof
		if err := publishPlanarManifold(budget, report, &a, &b, result, convexA, convexB, band); err != nil {
			return false, err
		}
	default:
		if report.Reason == reportvocab.ContactNoReason {
			report.Reason = reportvocab.ContactAmbiguousFeature
		}
	}
	return true, budget.Err()
}

// planarHintOf returns the nearest candidate pair the last exact planar
// relation of a, as the first solid, against b found, or the zero hint. A
// hint never changes a relation (planar.ClassifyPlanarHinted), so one left by a
// pose far from this one, or by a snapshot at another chord, only costs time.
func planarHintOf(a, b *topo.Body) planar.PlanarHint {
	stored, _ := a.PlanarHints.Load(b)
	hint, _ := stored.(planar.PlanarHint)
	return hint
}

// planarConvexity returns the body's cached §9.2 certificate, computing it at
// the identity pose on first use. A canceled computation caches nothing. A
// positive-displacement body's certificate is its held mesh's, read at chord
// when that mesh has a curved face.
func planarConvexity(ctx context.Context, budget *proofbound.WorkBudget, b *Body, chord float64) (bool, error) {
	bits := math.Float64bits(chord)
	if entry := b.n.PlanarConvexity.Load(); entry != nil && (entry.ChordFree || entry.ChordBits == bits) {
		return entry.Convex, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.OK {
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
	b.n.PlanarConvexity.Store(&planarsnapshot.ConvexityEntry{ChordBits: bits, ChordFree: snapshot.ChordFree, Convex: convex})
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
	if b == nil || !b.n.Solid || b.n.Kind != topo.BodySolid || !positiveAffine(pose) {
		return planar.PlanarSolid{}, none, false, nil
	}
	snapshot, err := planarSnapshotOf(ctx, budget, b, chord)
	if err != nil || !snapshot.OK {
		return planar.PlanarSolid{}, none, false, err
	}
	solid, ok, err := placePlanarSnapshot(budget, snapshot, pose)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	delta := snapshot.Delta
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
func placePlanarSnapshot(budget *proofbound.WorkBudget, snapshot *planarsnapshot.SnapshotEntry,
	pose r3.Transform) (planar.PlanarSolid, bool, error) {
	solid := planar.PlanarSolid{Verts: make([]proofarith.DyV3, len(snapshot.Solid.Verts)),
		Tris: slices.Clip(snapshot.Solid.Tris), Faces: slices.Clip(snapshot.Solid.Faces)}
	place := proofarith.NewDyadicMap(pose)
	for i, v := range snapshot.Solid.Verts {
		if err := budget.Step(); err != nil {
			return planar.PlanarSolid{}, false, err
		}
		solid.Verts[i] = place.Apply(v)
	}
	audited, err := planar.CheckPlanarPose(&solid, snapshot.Topology, budget.Step)
	if err != nil || !audited {
		return planar.PlanarSolid{}, false, err
	}
	return solid, true, nil
}

// planarSnapshotOf returns the body's exact snapshot at the identity query
// pose for chord, building and caching it on a miss. An error (cancellation,
// an exhausted budget) caches nothing; a refusal is cached like a snapshot,
// since it reads nothing but the body and the chord.
func planarSnapshotOf(ctx context.Context, budget *proofbound.WorkBudget, b *Body, chord float64) (*planarsnapshot.SnapshotEntry, error) {
	bits := math.Float64bits(chord)
	if entry := b.n.PlanarSnapshot.Load(); entry != nil && entry.ServesChord(bits) {
		return entry, nil
	}
	entry := &planarsnapshot.SnapshotEntry{ChordBits: bits, ChordFree: true, Delta: proofarith.DyZero()}
	var err error
	switch payload := b.n.Payload.(type) {
	case prismPayload:
		entry.Solid, entry.OK, err = planarPrismSolid(ctx, budget, payload, prismFaceIndex(b))
	case facetedPayload:
		entry.Solid, entry.Delta, entry.OK, err = planarFacetedSolid(budget, payload)
	case stitchPayload:
		entry.Solid, entry.Delta, entry.OK, err = planarStitchSolid(budget, b, payload)
	case coilPayload:
		entry.Solid, entry.Delta, entry.OK, err = planarCoilSolid(ctx, budget, b, payload)
	default:
		entry.Solid, entry.Delta, entry.OK, entry.ChordFree, err = planarHeldMeshSolid(ctx, budget, b, chord)
	}
	if err != nil {
		return nil, err
	}
	if entry.OK {
		entry.Topology, entry.OK, err = planar.NewPlanarTopology(len(entry.Solid.Verts), entry.Solid.Tris, budget.Step)
		if err != nil {
			return nil, err
		}
	}
	if !entry.OK {
		entry.Solid, entry.Delta = planar.PlanarSolid{}, proofarith.DyZero()
	}
	b.n.PlanarSnapshot.Store(entry)
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
	faces := b.Faces()
	faceRoles := make([][]string, len(faces))
	for i, face := range faces {
		for _, origin := range face.Origins() {
			faceRoles[i] = append(faceRoles[i], origin.Role)
		}
	}
	return selectorquery.RoleIndex(faceRoles)
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
		Outer: pp.Profile.Outer, Holes: pp.Profile.Holes,
		Frame: pp.Frame, Transform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1,
		SectionDelta: pp.SectionDelta, Z0Delta: pp.Z0Delta, Z1Delta: pp.Z1Delta,
		SurfaceResult: pp.SurfaceResult, CapStartRole: provenance.RoleCapStart, CapEndRole: provenance.RoleCapEnd,
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
		Verts: pp.Verts, Tris: pp.Tris, FaceOf: pp.FaceOf,
		ExactSourceVerts: pp.ExactSourceVerts, ExactSourceTris: pp.ExactSourceTris,
		MeshBound: pp.MeshBound, VolSymDiff: pp.VolSymDiff, Transform: pp.Xform,
	})
}

// planarStitchSolid reads a closed all-planar stitched solid off its own
// audited triangle set (§9). Stitch records Tris, TriFaces and VertBound only
// for an all-planar weld, and AuditClean only when the crossing audit ran and
// passed, so a curved or mixed stitch (nil Tris) and a failed audit are not
// admitted; planarSolidAtPose has already refused an open sheet, which is
// not a solid. The vertices are the welded table's floats at the stitch's
// placement, TriFaces names each triangle's live face, and a face missing
// from b.Faces() leaves the snapshot's face map unset.
//
// The displacement is the largest per-vertex weld bound, and at least the
// placement rounding Delta. Each VertBound entry is the bound vertexForClass
// stamps on its live Vertex, the weld class bound with Delta already added,
// and every held triangle point is a convex combination of its corners, so
// no true boundary point lies farther than that bound from the held mesh. It
// is zero for a weld built at the identity whose every class is zero-bound,
// which is then an exact §9 body.
func planarStitchSolid(budget *proofbound.WorkBudget, b *Body, sp stitchPayload) (planar.PlanarSolid, proofarith.Dyadic, bool, error) {
	none := proofarith.DyZero()
	if !sp.AuditClean || len(sp.Tris) == 0 || len(sp.TriFaces) != len(sp.Tris) ||
		len(sp.VertBound) != len(sp.Verts) || !measurement.FiniteValues(sp.Delta) || sp.Delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	displacement := sp.Delta
	for _, bound := range sp.VertBound {
		if !measurement.FiniteValues(bound) || bound < 0 {
			return planar.PlanarSolid{}, none, false, nil
		}
		displacement = max(displacement, bound)
	}
	faces := b.Faces()
	faceAt := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		faceAt[faceNode(face)] = i
	}
	faceOf := make([]int, len(sp.Tris))
	for t, face := range sp.TriFaces {
		at, ok := faceAt[face]
		if !ok {
			faceOf = nil
			break
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, sp.Verts, sp.Tris, faceOf)
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
	if !measurement.FiniteValues(cp.Delta) || cp.Delta < 0 {
		return planar.PlanarSolid{}, none, false, nil
	}
	mesh, err := tessellateCoil(ctx, b, cp, cp.Delta, VerifyNone)
	if err != nil {
		if ctxErr := ctx.Err(); ctxErr != nil {
			return planar.PlanarSolid{}, none, false, ctxErr
		}
		return planar.PlanarSolid{}, none, false, nil
	}
	faces := b.Faces()
	faceAt := make(map[*topo.Face]int, len(faces))
	for i, face := range faces {
		faceAt[faceNode(face)] = i
	}
	faceOf := make([]int, len(mesh.n.Triangles))
	for t, face := range mesh.n.Source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, mesh.n.Vertices, mesh.n.Triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, err
	}
	return solid, proofarith.MustDyOf(cp.Delta), true, nil
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
	faceAt := make(map[*topo.Face]int, len(faces))
	tolerance, chordFree := planarHeldChord, true
	for i, face := range faces {
		if !face.isPlanar() {
			tolerance, chordFree = chord, false
		}
		faceAt[faceNode(face)] = i
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
	if !mesh.n.BoundaryOK || !measurement.FiniteValues(mesh.n.Bound) || mesh.n.Bound < 0 ||
		len(mesh.n.Source) != len(mesh.n.Triangles) {
		return planar.PlanarSolid{}, none, false, chordFree, nil
	}
	faceOf := make([]int, len(mesh.n.Triangles))
	for t, face := range mesh.n.Source {
		at, ok := faceAt[face]
		if !ok {
			return planar.PlanarSolid{}, none, false, chordFree, nil
		}
		faceOf[t] = at
	}
	solid, ok, err := planarsnapshot.HeldSolid(budget, mesh.n.Vertices, mesh.n.Triangles, faceOf)
	if err != nil || !ok {
		return planar.PlanarSolid{}, none, false, chordFree, err
	}
	return solid, proofarith.MustDyOf(mesh.n.Bound), true, chordFree, nil
}
