package decad

import (
	"context"
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
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
	mesh := &Mesh{}
	spanWork := newCompositeSweepWork(sp.prism.profile)
	budget := proofbound.NewWorkBudget(ctx)
	coordinateBound := 0.0
	triangleSpan := []int{}
	join := make([][]int, len(sp.spans)+1)
	joinBound := make([][]float64, len(sp.spans)+1)
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
		part, err := span.build(ctx, b.doc, b.origin.producer, spanWork)
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
		local := make([]int, len(piece.vertices))
		for j := range local {
			local[j] = -1
		}
		maxMove := 0.0
		for station := range profileSamples {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			for _, boundary := range []struct {
				section int
				index   int
			}{{i, start[station]}, {i + 1, end[station]}} {
				shared := join[boundary.section][station]
				if shared < 0 {
					shared = len(mesh.vertices)
					mesh.vertices = append(mesh.vertices, piece.vertices[boundary.index])
					join[boundary.section][station] = shared
					joinBound[boundary.section][station] = vertexBounds[boundary.index]
				} else if mesh.vertices[shared] != piece.vertices[boundary.index] {
					delta2, ok := proofarith.DySquaredDistance3(
						mesh.vertices[shared].X, mesh.vertices[shared].Y, mesh.vertices[shared].Z,
						piece.vertices[boundary.index].X, piece.vertices[boundary.index].Y, piece.vertices[boundary.index].Z)
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
		for j, v := range piece.vertices {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			if local[j] < 0 {
				local[j] = len(mesh.vertices)
				mesh.vertices = append(mesh.vertices, v)
			}
		}
		coordinateBound = max(coordinateBound, proofbound.AbsSumUpper(spanCoordinateBound, maxMove))
		for t, triangle := range piece.triangles {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			role, err := compositeSpanFaceRole(piece.source[t], i, span.reverseArcCaps)
			if err != nil {
				return nil, err
			}
			if role == roleCapStart && (sp.surfaceResult || i != 0) ||
				role == roleCapEnd && (sp.surfaceResult || i != len(sp.spans)-1) {
				continue
			}
			face, ok := byRole[role]
			if !ok {
				return nil, fmt.Errorf(`%w: the composite sweep carries no face for role %q`, ErrDegenerate, role)
			}
			mesh.addTriangle([3]int{local[triangle[0]], local[triangle[1]], local[triangle[2]]}, face)
			if len(mesh.triangles) > maxCompositeMeshFacets {
				return nil, fmt.Errorf(`%w: the composite sweep mesh exceeds the fixed facet ceiling`, ErrUnsupported)
			}
			triangleSpan = append(triangleSpan, i)
			bound, ok := piece.sourceBound(piece.source[t])
			if !ok {
				return nil, fmt.Errorf(`%w: sweep path span %d has no source bound for role %q`, ErrUnsupported, i, role)
			}
			mesh.setFaceBound(face, proofbound.AbsSumUpper(bound, maxMove))
		}
		if verify >= VerifyAll {
			if !piece.symDiffOK {
				return nil, fmt.Errorf(`%w: sweep path span %d has no occupied-volume proof`, ErrUnsupported, i)
			}
			areaMove := revolvemesh.CoordinateAreaAllow(piece.vertices, piece.triangles, maxMove)
			volumeMove := proofbound.SweptVolumeAllow(maxMove,
				proofbound.PerturbedAreaUpper(piece.vertices, piece.triangles, maxMove))
			mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, piece.areaSlack, areaMove)
			mesh.volSymDiff = proofbound.AbsSumUpper(mesh.volSymDiff, piece.volSymDiff, volumeMove)
		}
	}
	if proofbound.IsNonFinite(mesh.bound) || proofbound.IsNonFinite(coordinateBound) ||
		proofbound.IsNonFinite(mesh.areaSlack) || proofbound.IsNonFinite(mesh.volSymDiff) {
		return nil, fmt.Errorf(`%w: the composite sweep mesh has no finite proof bound`, ErrUnsupported)
	}
	mesh.coordBound = coordinateBound
	if err := requireMeshAudit(ctx, sp.surfaceResult, b, mesh); err != nil {
		return nil, err
	}
	if !sp.surfaceResult {
		if err := liftTessellationError(tessellation.RequireVertexLinks(ctx, len(mesh.vertices), mesh.triangles)); err != nil {
			return nil, err
		}
	}
	auditBudget := proofbound.NewWorkBudget(ctx)
	auditTris, err := revolvemesh.RequireRevolveFacetAreas(auditBudget, mesh.vertices, mesh.triangles, coordinateBound)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyBoundary {
		if err := compositeCrossSpanContactAudit(auditBudget, auditTris, mesh.triangles, triangleSpan, coordinateBound); err != nil {
			return nil, err
		}
	}
	if !sp.surfaceResult && tessellation.OrientationSign(mesh.vertices, mesh.triangles, r3.Vec{}) <= 0 {
		return nil, fmt.Errorf(`%w: the composite sweep mesh does not enclose a positive volume`, ErrUnsupported)
	}
	mesh.symDiffOK = verify >= VerifyAll && !sp.surfaceResult
	return mesh, nil
}

func compositeSpanRings(ctx context.Context, span sweepSpanPayload, mesh *Mesh,
	samples int, freeformTarget float64) ([]int, []int, float64, error) {
	start := make([]int, samples)
	end := make([]int, samples)
	if !span.arc {
		if len(mesh.vertices) != 2*samples {
			return nil, nil, 0, fmt.Errorf(`%w: a straight sweep span has a different profile station chain`, ErrUnsupported)
		}
		for j := range samples {
			start[j], end[j] = 2*j, 2*j+1
		}
		return start, end, mesh.coordBound, nil
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
	if len(mesh.vertices)%samples != 0 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has an incomplete angular station chain`, ErrUnsupported)
	}
	angular := len(mesh.vertices) / samples
	if angular < 2 {
		return nil, nil, 0, fmt.Errorf(`%w: an arc sweep span has no angular cell`, ErrUnsupported)
	}
	for j := range samples {
		start[j], end[j] = j*angular, j*angular+angular-1
		if span.reverseArcCaps {
			start[j], end[j] = end[j], start[j]
		}
	}
	return start, end, mesh.coordBound, nil
}

// compositeCrossSpanContactAudit checks only pairs whose triangles came from
// different spans. Each reduced span already audited its own facet pairs at
// VerifyBoundary; the shared grid introduces only these new pairs.
func compositeCrossSpanContactAudit(budget *proofbound.WorkBudget, data []revolvemesh.RevolveAuditTri,
	triangles [][3]int, spanOf []int, delta float64) error {
	if len(data) != len(triangles) || len(spanOf) != len(triangles) {
		return fmt.Errorf(`%w: the composite sweep's facet audit has inconsistent triangle records`, ErrUnsupported)
	}
	pairs, ok := proofbound.WallChoose2(uint64(len(triangles)))
	if !ok || pairs > proofbound.MaxFacetPairTestsPerCall {
		return fmt.Errorf(`%w: the composite sweep's facet-pair audit exceeds its fixed work ceiling`, ErrUnsupported)
	}
	margin := proofbound.ProductUpper(2, delta)
	for i := range triangles {
		for j := i + 1; j < len(triangles); j++ {
			if spanOf[i] == spanOf[j] {
				continue
			}
			if err := budget.Step(); err != nil {
				return err
			}
			shared, count := tessellation.SharedVertexIndices(triangles[i], triangles[j])
			if count == 0 && revolvemesh.BoxGapExceeds(data[i].Box, data[j].Box, margin) {
				continue
			}
			if err := revolvemesh.AuditRevolvePair(data, triangles, shared, count, i, j, delta); err != nil {
				return err
			}
		}
	}
	return budget.Err()
}

func compositeSpanFaceRole(face *Face, span int, reverseCaps bool) (string, error) {
	origins := face.Origins()
	if len(origins) == 0 {
		return "", fmt.Errorf(`%w: a sweep span face has no origin role`, ErrDegenerate)
	}
	role := origins[0].Role
	if suffix, ok := strings.CutPrefix(role, "side("); ok {
		return fmt.Sprintf("side(%d,%s", span, suffix), nil
	}
	if reverseCaps {
		switch role {
		case roleCapStart:
			return roleCapEnd, nil
		case roleCapEnd:
			return roleCapStart, nil
		}
	}
	return role, nil
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
	loops := append([]loopRecord{sp.prism.profile.Outer}, sp.prism.profile.Holes...)
	counts := map[compositeStationKey]int{}
	work := newCompositeSweepWork(sp.prism.profile)
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
			Resolution: res, Chord: chord, SectionDelta: span.revolve.sectionDelta,
			Phi0: span.revolve.phi0, Phi1: span.revolve.phi1, Full: span.revolve.full,
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
