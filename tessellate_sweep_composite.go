package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/compositesweep"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

const maxCompositeMeshFacets = 65_536

// tessellateCompositeSweep assembles one indexed profile grid across the
// reduced spans. The station adapter admits recorded lines and circular walks
// and fitted splines whose span meshes expose the same profile parameter at
// each join. It maps those samples by profile index, never spatial proximity.
func tessellateCompositeSweep(ctx context.Context, b *Body, sp sweepPayload, chord float64, verify Verification) (*Mesh, error) {
	counts, profileSamples, freeformTarget, err := compositeProfileCountPlan(ctx, sp, chord)
	if err != nil {
		return nil, err
	}
	byRole := map[string]*Face{}
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			byRole[origin.Role] = f
		}
	}
	mesh := bornMesh(&Mesh{})
	spanWork := newCompositeSweepWork(sp.prism.Profile)
	budget := proofbound.NewWorkBudget(ctx)
	coordinateBound := 0.0
	triangleSpan := []int{}
	stations := len(sp.spans) + 1
	if sp.closedComposite {
		stations = len(sp.spans)
	}
	join := make([][]int, stations)
	joinBound := make([][]float64, stations)
	for i := range join {
		join[i] = make([]int, profileSamples)
		joinBound[i] = make([]float64, profileSamples)
		for j := range join[i] {
			join[i][j] = -1
		}
	}
	for i := range sp.spans {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		span := sp.spans[i]
		part, err := span.build(ctx, documentHandle(b.n.Doc), b.n.Origin.Producer, spanWork)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		// span.build restores path cap names for topology pairing. The reduced
		// revolve tessellator needs its own cap names while it makes its proof.
		if span.arc && span.reverseArcCaps {
			restoreSweepArcCapRoles(part, true)
		}
		var piece *Mesh
		if span.arc {
			floor, floorErr := compositeRevolveFloor(ctx, span.revolve, counts, freeformTarget)
			if floorErr != nil {
				return nil, fmt.Errorf(`sweep path span %d: %w`, i, floorErr)
			}
			piece, err = tessellateRevolveWithCountFloor(ctx, part, span.revolve, chord, verify, nil, floor, freeformTarget)
		} else {
			piece, err = tessellatePrismWithCountFloor(ctx, part, span.prism, prismWallRole, chord, verify,
				func(loop, seg int) int { return counts[compositeStationKey{loop, seg}] }, freeformTarget)
		}
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		vertexBounds, err := piece.vertexBounds()
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		start, end, spanCoordinateBound, err := compositeSpanRings(ctx, span, piece, profileSamples, freeformTarget)
		if err != nil {
			return nil, fmt.Errorf(`sweep path span %d: %w`, i, err)
		}
		local := make([]int, len(piece.n.Vertices))
		for j := range local {
			local[j] = -1
		}
		maxMove := 0.0
		endSection := i + 1
		if sp.closedComposite && endSection == len(sp.spans) {
			endSection = 0
		}
		for station := range profileSamples {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			for _, boundary := range []struct {
				section int
				index   int
			}{{i, start[station]}, {endSection, end[station]}} {
				shared := join[boundary.section][station]
				if shared < 0 {
					shared = len(mesh.n.Vertices)
					mesh.n.Vertices = append(mesh.n.Vertices, piece.n.Vertices[boundary.index])
					join[boundary.section][station] = shared
					joinBound[boundary.section][station] = vertexBounds[boundary.index]
				} else if mesh.n.Vertices[shared] != piece.n.Vertices[boundary.index] {
					delta2, ok := proofarith.DySquaredDistance3(
						mesh.n.Vertices[shared].X, mesh.n.Vertices[shared].Y, mesh.n.Vertices[shared].Z,
						piece.n.Vertices[boundary.index].X, piece.n.Vertices[boundary.index].Y, piece.n.Vertices[boundary.index].Z)
					if !ok {
						return nil, fmt.Errorf(`%w: a shared sweep station has no finite displacement`, ErrUnsupported)
					}
					move := proofarith.DySqrtUp(delta2)
					if proofbound.IsNonFinite(move) ||
						move > proofbound.AbsSumUpper(joinBound[boundary.section][station], vertexBounds[boundary.index]) {
						return nil, fmt.Errorf(`%w: adjacent sweep spans disagree on profile station %d at section %d`, ErrUnsupported, station, boundary.section)
					}
					maxMove = max(maxMove, move)
				}
				local[boundary.index] = shared
			}
		}
		for j, v := range piece.n.Vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if local[j] < 0 {
				local[j] = len(mesh.n.Vertices)
				mesh.n.Vertices = append(mesh.n.Vertices, v)
			}
		}
		coordinateBound = max(coordinateBound, proofbound.AbsSumUpper(spanCoordinateBound, maxMove))
		for t, triangle := range piece.n.Triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			role, err := compositeSpanFaceRole(piece.n.Source[t], i, span.reverseArcCaps)
			if err != nil {
				return nil, err
			}
			if sp.closedComposite && (role == provenance.RoleCapStart || role == provenance.RoleCapEnd) ||
				role == provenance.RoleCapStart && (sp.surfaceResult || i != 0) ||
				role == provenance.RoleCapEnd && (sp.surfaceResult || i != len(sp.spans)-1) {
				continue
			}
			face, ok := byRole[role]
			if !ok {
				return nil, fmt.Errorf(`%w: the composite sweep carries no face for role %q`, ErrDegenerate, role)
			}
			topobuild.AddTriangle(&mesh.n, [3]int{local[triangle[0]], local[triangle[1]], local[triangle[2]]}, faceNode(face))
			if len(mesh.n.Triangles) > maxCompositeMeshFacets {
				return nil, fmt.Errorf(`%w: the composite sweep mesh exceeds the fixed facet ceiling`, ErrUnsupported)
			}
			triangleSpan = append(triangleSpan, i)
			bound, ok := piece.sourceBound(piece.n.Source[t])
			if !ok {
				return nil, fmt.Errorf(`%w: sweep path span %d has no source bound for role %q`, ErrUnsupported, i, role)
			}
			topobuild.SetFaceBound(&mesh.n, faceNode(face), proofbound.AbsSumUpper(bound, maxMove))
		}
		if verify >= VerifyAll {
			if !piece.n.SymDiffOK {
				return nil, fmt.Errorf(`%w: sweep path span %d has no occupied-volume proof`, ErrUnsupported, i)
			}
			areaMove := revolvemesh.CoordinateAreaAllow(piece.n.Vertices, piece.n.Triangles, maxMove)
			volumeMove := proofbound.SweptVolumeAllow(maxMove,
				proofbound.PerturbedAreaUpper(piece.n.Vertices, piece.n.Triangles, maxMove))
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, piece.n.AreaSlack, areaMove)
			mesh.n.VolSymDiff = proofbound.AbsSumUpper(mesh.n.VolSymDiff, piece.n.VolSymDiff, volumeMove)
		}
	}
	if proofbound.IsNonFinite(mesh.n.Bound) || proofbound.IsNonFinite(coordinateBound) ||
		proofbound.IsNonFinite(mesh.n.AreaSlack) || proofbound.IsNonFinite(mesh.n.VolSymDiff) {
		return nil, fmt.Errorf(`%w: the composite sweep mesh has no finite proof bound`, ErrUnsupported)
	}
	mesh.n.CoordBound = coordinateBound
	if err := requireMeshAudit(ctx, sp.surfaceResult, b, mesh); err != nil {
		return nil, err
	}
	if !sp.surfaceResult {
		if err := topobuild.LiftAuditError(tessellation.RequireVertexLinks(ctx, len(mesh.n.Vertices), mesh.n.Triangles)); err != nil {
			return nil, err
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(auditBudget, mesh.n.Vertices, mesh.n.Triangles, coordinateBound)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := compositesweep.CrossSpanContactAudit(auditBudget, auditTris, mesh.n.Triangles, triangleSpan, coordinateBound); err != nil {
			return nil, err
		}
	}
	if !sp.surfaceResult && tessellation.OrientationSign(mesh.n.Vertices, mesh.n.Triangles, r3.Vec{}) <= 0 {
		return nil, fmt.Errorf(`%w: the composite sweep mesh does not enclose a positive volume`, ErrUnsupported)
	}
	mesh.n.SymDiffOK = verify >= VerifyAll && !sp.surfaceResult
	return mesh, nil
}

func compositeSpanRings(ctx context.Context, span sweepSpanPayload, mesh *Mesh,
	samples int, freeformTarget float64) ([]int, []int, float64, error) {
	start := make([]int, samples)
	end := make([]int, samples)
	if !span.arc {
		if len(mesh.n.Vertices) != 2*samples {
			return nil, nil, 0, fmt.Errorf(`%w: a straight sweep span has a different profile station chain`, ErrUnsupported)
		}
		for j := range samples {
			start[j], end[j] = 2*j, 2*j+1
		}
		return start, end, mesh.n.CoordBound, nil
	}
	res, err := resolveRevolveWithLimit(ctx, span.revolve, compositeWorkLimit(freeformTarget))
	if err != nil {
		return nil, nil, 0, err
	}
	for _, loop := range res.Junctions {
		for _, junction := range loop {
			if junction.OnAxis {
				return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span's pole needs a shared station plan`, ErrUnsupported)
			}
		}
	}
	if len(mesh.n.Vertices)%samples != 0 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has an incomplete angular station chain`, ErrUnsupported)
	}
	angular := len(mesh.n.Vertices) / samples
	if angular < 2 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has no angular cell`, ErrUnsupported)
	}
	for j := range samples {
		start[j], end[j] = j*angular, j*angular+angular-1
		if span.reverseArcCaps {
			start[j], end[j] = end[j], start[j]
		}
	}
	return start, end, mesh.n.CoordBound, nil
}

func compositeSpanFaceRole(face *topo.Face, span int, reverseCaps bool) (string, error) {
	origins := face.Origins
	if len(origins) == 0 {
		return "", fmt.Errorf(`%w: a sweep span face has no origin role`, ErrDegenerate)
	}
	return compositesweep.SpanFaceRole(origins[0].Role, span, reverseCaps, provenance.RoleCapStart, provenance.RoleCapEnd), nil
}

type compositeStationKey struct{ loop, segment int }

func compositeWorkLimit(freeformTarget float64) uint64 {
	if freeformTarget > 0 {
		return compositeFitSweepWorkLimit
	}
	return 0
}

// compositeProfileCountPlan chooses one parameter count per recorded profile
// segment before any span emits vertices. Each circular reduction's own count
// is a lower bound. Fitted splines instead share the smallest certified
// chording target across their prism and Revolve spans.
func compositeProfileCountPlan(ctx context.Context, sp sweepPayload, chord float64) (map[compositeStationKey]int, int, float64, error) {
	loops := append([]loopRecord{sp.prism.Profile.Outer}, sp.prism.Profile.Holes...)
	counts := map[compositeStationKey]int{}
	work := newCompositeSweepWork(sp.prism.Profile)
	freeformTarget := chord
	hasFreeform := false
	for li, loop := range loops {
		for si, seg := range loop.Segments {
			if err := ctx.Err(); err != nil {
				return nil, 0, 0, err
			}
			key := compositeStationKey{li, si}
			switch seg.(type) {
			case sectionrecord.LineSeg:
				counts[key] = 1
			case sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				walk, err := boundarywalk.WalkOf(seg, work)
				if err != nil {
					return nil, 0, 0, err
				}
				n, _, err := tessellation.ChordCount(walk, chord, tessellation.ChordWalkMin(walk))
				if err != nil {
					return nil, 0, 0, err
				}
				counts[key] = n
			case sectionrecord.FitSplineSeg:
				hasFreeform = true
			default:
				return nil, 0, 0, fmt.Errorf(`%w: composite sweep mesh profile stations do not cover segment %T`, ErrUnsupported, seg)
			}
		}
	}
	for _, span := range sp.spans {
		if !span.arc {
			continue
		}
		workLimit := uint64(0)
		if hasFreeform {
			workLimit = compositeFitSweepWorkLimit
		}
		res, err := resolveRevolveWithLimit(ctx, span.revolve, workLimit)
		if err != nil {
			return nil, 0, 0, err
		}
		planned, err := revolveplan.PlanCounts(revolveplan.CountInput{
			Resolution: res, Chord: chord, SectionDelta: span.revolve.SectionDelta,
			Phi0: span.revolve.Phi0, Phi1: span.revolve.Phi1, Full: span.revolve.Full,
			WorkLimit: workLimit,
		})
		if err != nil {
			return nil, 0, 0, err
		}
		freeformTarget = min(freeformTarget, planned.MeridianTarget)
		for li, loop := range res.Resolved {
			for wi, walk := range loop.Walks {
				key := compositeStationKey{li, walk.Segs[0]}
				counts[key] = max(counts[key], planned.Meridian[li][wi])
			}
		}
	}
	if hasFreeform {
		for li, loop := range loops {
			for si, seg := range loop.Segments {
				if _, ok := seg.(sectionrecord.FitSplineSeg); !ok {
					continue
				}
				walk, err := boundarywalk.WalkOf(seg, work)
				if err != nil {
					return nil, 0, 0, err
				}
				chain, err := freeform.ChainStations(walk.Spans, freeformTarget, work)
				if err != nil {
					return nil, 0, 0, err
				}
				counts[compositeStationKey{li, si}] = len(chain.Stations)
			}
		}
	} else {
		freeformTarget = 0
	}
	total := 0
	for li, loop := range loops {
		loopSamples := 0
		for si := range loop.Segments {
			count := counts[compositeStationKey{li, si}]
			if count > maxCompositeMeshFacets-loopSamples {
				return nil, 0, 0, fmt.Errorf(`%w: the composite sweep profile exceeds the fixed mesh station ceiling`, ErrUnsupported)
			}
			loopSamples += count
		}
		if loopSamples < 3 {
			return nil, 0, 0, fmt.Errorf(`%w: a composite sweep mesh loop needs at least three profile samples`, ErrDegenerate)
		}
		if loopSamples > maxCompositeMeshFacets-total {
			return nil, 0, 0, fmt.Errorf(`%w: the composite sweep profile exceeds the fixed mesh station ceiling`, ErrUnsupported)
		}
		total += loopSamples
	}
	return counts, total, freeformTarget, nil
}

func compositeRevolveFloor(ctx context.Context, rp revolvePayload,
	counts map[compositeStationKey]int, freeformTarget float64) ([][]int, error) {
	res, err := resolveRevolveWithLimit(ctx, rp, compositeWorkLimit(freeformTarget))
	if err != nil {
		return nil, err
	}
	floor := make([][]int, len(res.Resolved))
	for li, loop := range res.Resolved {
		floor[li] = make([]int, len(loop.Walks))
		for wi, walk := range loop.Walks {
			key := compositeStationKey{li, walk.Segs[0]}
			count, ok := counts[key]
			if !ok {
				return nil, fmt.Errorf(`%w: an arc sweep meridian has no shared profile station count`, ErrUnsupported)
			}
			floor[li][wi] = count
		}
	}
	return floor, nil
}
