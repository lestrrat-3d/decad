package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftcylinder"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/triangulation"
	"github.com/lestrrat-3d/r3"
)

type loftCylinderChamferSide struct {
	groups    [][2]int
	clipBound float64
	rings     [2][]int
	exact     [2][]proofarith.Xpt
}

func loftCylinderTriangleKey(tri [3]int) [3]int {
	sort.Ints(tri[:])
	return tri
}

// The joined shell is the paired two-triangle loft strip of its recorded cap
// rings. The bore, when present, owns a separate wall group and stays intact.
func loftCylinderChamferSideGroups(fp facetedPayload, source loftCylinderChamferSource) ([][2]int, error) {
	n := len(source.outer[0])
	if n < 3 || len(source.outer[1]) != n || len(fp.src) != len(fp.tris) {
		return nil, fmt.Errorf("%w: loft/cylinder side lacks paired cap rings", ErrUnsupported)
	}
	lookup := make(map[[3]int]int, 2*n)
	for i, tri := range fp.tris {
		group := fp.src[i]
		if group == source.capGroups[0] || group == source.capGroups[1] || group == source.boreGroup {
			continue
		}
		key := loftCylinderTriangleKey(tri)
		if _, duplicate := lookup[key]; duplicate {
			return nil, fmt.Errorf("%w: loft/cylinder side repeats a triangle", ErrUnsupported)
		}
		lookup[key] = group
	}
	groups := make([][2]int, n)
	for i := range n {
		j := (i + 1) % n
		b0, b1 := source.outer[0][i], source.outer[0][j]
		t0, t1 := source.outer[1][i], source.outer[1][j]
		for _, index := range [4]int{b0, b1, t0, t1} {
			if index < 0 || index >= len(fp.verts) {
				return nil, fmt.Errorf("%w: loft/cylinder ring index is invalid", ErrUnsupported)
			}
		}
		first := loftCylinderTriangleKey([3]int{b0, b1, t1})
		second := loftCylinderTriangleKey([3]int{b0, t1, t0})
		g0, ok0 := lookup[first]
		g1, ok1 := lookup[second]
		if !ok0 || !ok1 {
			return nil, fmt.Errorf("%w: loft/cylinder side lost its two-triangle strip", ErrUnsupported)
		}
		groups[i] = [2]int{g0, g1}
		delete(lookup, first)
		delete(lookup, second)
	}
	if len(lookup) != 0 {
		return nil, fmt.Errorf("%w: loft/cylinder side has an unmatched facet", ErrUnsupported)
	}
	return groups, nil
}

func loftCylinderChamferLerp(a, b r3.Vec, level *big.Rat, heldLevel float64) (r3.Vec, proofarith.Xpt, float64, error) {
	den := new(big.Rat).Sub(proofarith.FloatRat(b.Z), proofarith.FloatRat(a.Z))
	if den.Sign() == 0 {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: loft/cylinder side has zero axial span", ErrUnsupported)
	}
	t := new(big.Rat).Quo(new(big.Rat).Sub(level, proofarith.FloatRat(a.Z)), den)
	if t.Sign() <= 0 || t.Cmp(big.NewRat(1, 1)) >= 0 {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: chamfer side level lies outside a wall", ErrUnsupported)
	}
	exact := proofarith.Xlerp(proofarith.XptOf(a), proofarith.XptOf(b), t.Num(), t.Denom())
	p := exact.Vec()
	p.Z = heldLevel
	if !proofbound.FiniteVec(p) {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: chamfer side station is nonfinite", ErrUnsupported)
	}
	round := max(
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 0), p.X),
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 1), p.Y),
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 2), p.Z))
	return p, exact, proofbound.Radius3D(round), nil
}

// Side rings alternate each source ruling with its old triangle diagonal.
// That is the exact horizontal section of the old two-triangle strip before
// each intersection point is rounded into the result mesh.
func loftCylinderChamferClipSide(ctx context.Context, mesh *joinedMesh,
	vertexBound *[]float64, fp facetedPayload, source loftCylinderChamferSource,
	groups [][2]int, d float64) (loftCylinderChamferSide, error) {
	n := len(groups)
	result := loftCylinderChamferSide{groups: groups}
	levels := [2]float64{source.z0 + d, source.z1 - d}
	trueLevels := [2]*big.Rat{
		new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(d)),
		new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(d)),
	}
	for cap, level := range levels {
		result.rings[cap] = make([]int, 2*n)
		result.exact[cap] = make([]proofarith.Xpt, 2*n)
		for i := range n {
			if i%64 == 0 && ctx.Err() != nil {
				return result, ctx.Err()
			}
			b := source.outer[0][i]
			t := source.outer[1][i]
			next := source.outer[1][(i+1)%n]
			for k, end := range [2]int{t, next} {
				p, exact, round, err := loftCylinderChamferLerp(fp.verts[b], fp.verts[end],
					trueLevels[cap], level)
				if err != nil {
					return result, err
				}
				result.clipBound = max(result.clipBound, round)
				result.rings[cap][2*i+k] = len(mesh.verts)
				result.exact[cap][2*i+k] = exact
				mesh.verts = append(mesh.verts, p)
				*vertexBound = append(*vertexBound, proofbound.AbsSumUpper(fp.meshBound, round))
			}
		}
	}
	lo, hi := result.rings[0], result.rings[1]
	for i, group := range groups {
		j := (i + 1) % n
		// Original [b_i,b_j,t_j].
		mesh.add([3]int{lo[2*i+1], lo[2*j], hi[2*j]}, group[0])
		mesh.add([3]int{lo[2*i+1], hi[2*j], hi[2*i+1]}, group[0])
		// Original [b_i,t_j,t_i].
		mesh.add([3]int{lo[2*i], lo[2*i+1], hi[2*i+1]}, group[1])
		mesh.add([3]int{lo[2*i], hi[2*i+1], hi[2*i]}, group[1])
	}
	return result, nil
}

func loftCylinderChamferTriangulate(ctx context.Context, mesh *joinedMesh,
	outer, inner []int, group int, bottom bool) error {
	points := make([]triangulation.Point2, 0, len(outer)+len(inner))
	outerLocal := make([]int, len(outer))
	for i, index := range outer {
		if index < 0 || index >= len(mesh.verts) {
			return fmt.Errorf("%w: cap band has an invalid outer vertex", ErrUnsupported)
		}
		p := mesh.verts[index]
		outerLocal[i] = len(points)
		points = append(points, triangulation.Point2{U: p.X, V: p.Y})
	}
	loops := [][]int{outerLocal}
	if len(inner) > 0 {
		hole := make([]int, len(inner))
		for i := range inner {
			index := inner[len(inner)-1-i]
			if index < 0 || index >= len(mesh.verts) {
				return fmt.Errorf("%w: cap band has an invalid inner vertex", ErrUnsupported)
			}
			p := mesh.verts[index]
			hole[i] = len(points)
			points = append(points, triangulation.Point2{U: p.X, V: p.Y})
		}
		loops = append(loops, hole)
	}
	tris, err := triangulation.Triangulate(ctx, points, loops)
	if err != nil {
		return triangulation.WrapLoftError(err)
	}
	local := append(append([]int(nil), outer...), reverseInts(inner)...)
	for _, tri := range tris {
		mapped := [3]int{local[tri[0]], local[tri[1]], local[tri[2]]}
		if bottom {
			mapped[1], mapped[2] = mapped[2], mapped[1]
		}
		mesh.add(mapped, group)
	}
	return nil
}

// Each old source edge has one offset line. Its side section has a ruling,
// diagonal, then the next ruling. These local strips preserve that order;
// a reflex offset arc receives a fan only at its own source corner.
func loftCylinderChamferBand(mesh *joinedMesh, side, inset []int,
	samples loftCylinderInsetSamples, group int) error {
	n := len(samples.lineStart)
	if len(side) != 2*n || len(samples.arcStart) != n || len(inset) != len(samples.points) {
		return fmt.Errorf("%w: loft/cylinder band has unmatched source pieces", ErrUnsupported)
	}
	for i := range n {
		j := (i + 1) % n
		q, diag, next := side[2*i], side[2*i+1], side[2*j]
		line := inset[samples.lineStart[i]]
		end := inset[samples.lineStart[j]]
		if samples.arcStart[j] >= 0 {
			end = inset[samples.arcStart[j]]
		}
		mesh.add([3]int{diag, next, end}, group)
		mesh.add([3]int{diag, end, line}, group)
		mesh.add([3]int{q, diag, line}, group)
		if samples.arcStart[i] < 0 {
			continue
		}
		// Walk the arc backwards from the next offset line's start. The
		// first triangle above already owns the final arc-to-line segment.
		current := samples.lineStart[i]
		for current != samples.arcStart[i] {
			previous := (current - 1 + len(inset)) % len(inset)
			mesh.add([3]int{q, inset[current], inset[previous]}, group)
			current = previous
		}
	}
	return nil
}

func reverseInts(input []int) []int {
	out := make([]int, len(input))
	for i := range input {
		out[i] = input[len(input)-1-i]
	}
	return out
}

func loftCylinderChamferContactAudit(ctx context.Context, fp facetedPayload,
	mesh *joinedMesh, source loftCylinderChamferSource, side loftCylinderChamferSide,
	bands [2][2]int, sideEdges [2]map[[2]int]struct{},
	xverts []proofarith.Xpt, setback float64) error {
	oldData := loftmesh.NewLoftAuditData(fp.verts, fp.tris)
	oldParity := meshbool.NewParityMesh(fp.verts, fp.tris)
	allTris := make([]int, len(fp.tris))
	for i := range allTris {
		allTris[i] = i
	}
	oldBoxes := make([][2]r3.Vec, len(fp.tris))
	for i, tri := range fp.tris {
		oldBoxes[i] = meshbool.TriBox(fp.verts, tri)
	}
	for cap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		pointTouches := make(map[string]struct{})
		segmentEnds := make(map[string]struct{})
		sideVertexKeys := make(map[string]struct{}, len(side.rings[cap]))
		for _, vertex := range side.rings[cap] {
			sideVertexKeys[xverts[vertex].Key()] = struct{}{}
		}
		tri := mesh.tris[band[0]]
		witness := meshbool.XCentroid(xverts[tri[0]], xverts[tri[1]], xverts[tri[2]])
		inside, on, parityErr := meshbool.MeshParityPreparedContext(ctx, witness, oldParity, allTris)
		if parityErr != nil {
			return parityErr
		}
		if !inside || on {
			return fmt.Errorf("%w: ideal cap band is not inside the old solid", ErrUnsupported)
		}
		level := source.z0 + setback
		trueLevel := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(setback))
		if cap == 1 {
			level = source.z1 - setback
			trueLevel = new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(setback))
		}
		for i := band[0]; i < band[1]; i++ {
			if i%64 == 0 {
				if err := ctx.Err(); err != nil {
					return err
				}
			}
			box := meshbool.TriBox(mesh.verts, mesh.tris[i])
			pad := max(2*side.clipBound, 1e-12)
			box[0] = r3.Vec{X: math.Nextafter(box[0].X-pad, math.Inf(-1)),
				Y: math.Nextafter(box[0].Y-pad, math.Inf(-1)), Z: math.Nextafter(box[0].Z-pad, math.Inf(-1))}
			box[1] = r3.Vec{X: math.Nextafter(box[1].X+pad, math.Inf(1)),
				Y: math.Nextafter(box[1].Y+pad, math.Inf(1)), Z: math.Nextafter(box[1].Z+pad, math.Inf(1))}
			tri := mesh.tris[i]
			var xa [3]proofarith.Xpt
			for k, index := range tri {
				xa[k] = xverts[index]
			}
			na := proofarith.Xcross(proofarith.Xsub(xa[1], xa[0]), proofarith.Xsub(xa[2], xa[0]))
			fa := [3]r3.Vec{xa[0].Vec(), xa[1].Vec(), xa[2].Vec()}
			bandPlane := loftmesh.NewLoftExactPlane(xa[0], na)
			for j := range fp.tris {
				if fp.src[j] == source.capGroups[0] || fp.src[j] == source.capGroups[1] ||
					fp.src[j] == source.boreGroup || !meshbool.BoxesOverlap(box, oldBoxes[j]) {
					continue
				}
				signsA := loftmesh.TrianglePlaneSigns(oldData.Planes[j], xa)
				signsB := loftmesh.TrianglePlaneSigns(bandPlane, oldData.Xtris[j])
				contact, err := meshbool.TriTriClassifyCore(fa, oldData.Corners[j], xa,
					oldData.Xtris[j], na, oldData.Norms[j], nil, nil, &signsA, &signsB,
					false, r3.Vec{}, r3.Vec{}, false)
				if err != nil {
					return err
				}
				if contact.Kind == meshbool.ContactPoint {
					pointTouches[contact.P0.Key()] = struct{}{}
					_, atSideVertex := sideVertexKeys[contact.P0.Key()]
					if meshbool.RatCoordOf(contact.P0, 2).Cmp(trueLevel) != 0 ||
						!contact.P0OnA ||
						(!atSideVertex && !loftCylinderChamferContactOnSide(
							contact.P0, tri, xverts, sideEdges[cap])) {
						return fmt.Errorf("%w: ideal cap band touches an old wall away from its side ring: cap=%d band=%d old=%d point=%v tri=%v level=%g",
							ErrUnsupported, cap, i, j, contact.P0.Vec(), tri, level)
					}
					continue
				}
				if contact.Kind == meshbool.ContactRegion {
					return fmt.Errorf("%w: ideal cap band overlaps an old wall", ErrUnsupported)
				}
				if contact.Kind != meshbool.ContactSegment {
					continue
				}
				segmentEnds[contact.P0.Key()] = struct{}{}
				segmentEnds[contact.P1.Key()] = struct{}{}
				if contact.EdgeA < 0 ||
					meshbool.RatCoordOf(contact.P0, 2).Cmp(trueLevel) != 0 ||
					meshbool.RatCoordOf(contact.P1, 2).Cmp(trueLevel) != 0 {
					return fmt.Errorf("%w: ideal cap band crosses an old wall away from its side ring", ErrUnsupported)
				}
				a, b := tri[contact.EdgeA], tri[(contact.EdgeA+1)%3]
				if _, ok := sideEdges[cap][loftCylinderChamferEdge(a, b)]; !ok {
					return fmt.Errorf("%w: ideal cap band touches an old wall on a non-rim edge", ErrUnsupported)
				}
			}
		}
		for point := range pointTouches {
			if _, segment := segmentEnds[point]; !segment {
				return fmt.Errorf("%w: loft/cylinder cap has an isolated wall tangency", ErrUnsupported)
			}
		}
	}
	return nil
}

// Both rings are simple and counterclockwise on entry. Exact segment tests
// plus one strict inside witness prove the whole inner ring is nested.
func loftCylinderChamferRingNested(ctx context.Context, verts []r3.Vec, outer, inner []int) (bool, error) {
	if len(outer) < 3 || len(inner) < 3 {
		return false, nil
	}
	for i, a := range outer {
		b := outer[(i+1)%len(outer)]
		pa, pb := verts[a], verts[b]
		for j, c := range inner {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			d := inner[(j+1)%len(inner)]
			pc, pd := verts[c], verts[d]
			if max(pa.X, pb.X) < min(pc.X, pd.X) || max(pc.X, pd.X) < min(pa.X, pb.X) ||
				max(pa.Y, pb.Y) < min(pc.Y, pd.Y) || max(pc.Y, pd.Y) < min(pa.Y, pb.Y) {
				continue
			}
			if loftCylinderEdgesMeet(loftCylinderExactXY(pa), loftCylinderExactXY(pb),
				loftCylinderExactXY(pc), loftCylinderExactXY(pd)) {
				return false, fmt.Errorf("%w: cap rings meet at outer edge %d (%v,%v) and inner edge %d (%v,%v)",
					ErrUnsupported, i, pa, pb, j, pc, pd)
			}
		}
	}
	poly := make([]meshbool.Xp2, len(outer))
	for i, index := range outer {
		p := verts[index]
		poly[i] = meshbool.NewXP2(proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y))
	}
	p := verts[inner[0]]
	inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), poly,
		meshbool.NewXP2(proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)))
	return inside && !on, err
}

func loftCylinderChamferCompact(mesh *joinedMesh, bounds []float64) ([]float64, []int) {
	used := make([]bool, len(mesh.verts))
	for _, tri := range mesh.tris {
		for _, index := range tri {
			used[index] = true
		}
	}
	index := make([]int, len(mesh.verts))
	verts := make([]r3.Vec, 0, len(mesh.verts))
	newBounds := make([]float64, 0, len(bounds))
	for i, yes := range used {
		if !yes {
			continue
		}
		index[i] = len(verts)
		verts = append(verts, mesh.verts[i])
		newBounds = append(newBounds, bounds[i])
	}
	for i := range mesh.tris {
		for k := range 3 {
			mesh.tris[i][k] = index[mesh.tris[i][k]]
		}
	}
	mesh.verts = verts
	return newBounds, index
}

type loftCylinderOneCapData struct {
	capGroups [2]int
	outer     [2][]int
	inner     [2][]int
	boreGroup int
	ideal     []proofarith.Xpt
	band      [2]int
}

// The top of a private one-cap rewrite is an exact subset of the old side.
// The split ring came from the same two-triangle strip as the middle side;
// joining it to the original top stations leaves the middle cap unchanged.
func loftCylinderChamferRestoreTop(mesh *joinedMesh, fp facetedPayload,
	source loftCylinderChamferSource, groups [][2]int, ring []int) error {
	n := len(groups)
	if len(ring) != 2*n || len(source.outer[1]) != n {
		return fmt.Errorf("%w: one-cap chamfer lost its top side ring", ErrUnsupported)
	}
	for i, pair := range groups {
		j := (i + 1) % n
		topI, topJ := source.outer[1][i], source.outer[1][j]
		mesh.add([3]int{ring[2*i+1], ring[2*j], topJ}, pair[0])
		mesh.add([3]int{ring[2*i], ring[2*i+1], topJ}, pair[1])
		mesh.add([3]int{ring[2*i], topJ, topI}, pair[1])
	}
	for i, group := range fp.src {
		if group == source.capGroups[1] {
			mesh.add(fp.tris[i], group)
		}
	}
	return nil
}

func buildLoftCylinderCapChamfer(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback float64) (*Body, error) {
	return buildLoftCylinderCapChamferCaps(ctx, d, ref, fp, source, setback,
		[2]bool{true, true}, 0.1, nil, nil)
}

// The inactive top cap is kept verbatim, including its bore rim and middle
// annulus. The active bottom band uses the same fixed-cutter proof as the
// public two-cap route. This private result is sewn to its reflection.
func buildLoftCylinderOneCapChamfer(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback, allowance float64,
	reflectedShift []*big.Rat, data *loftCylinderOneCapData) (*Body, error) {
	return buildLoftCylinderCapChamferCaps(ctx, d, ref, fp, source, setback,
		[2]bool{true, false}, allowance, reflectedShift, data)
}

func buildLoftCylinderCapChamferCaps(ctx context.Context, d *Document, ref producerID,
	fp facetedPayload, source loftCylinderChamferSource, setback float64,
	active [2]bool, allowance float64, reflectedShift []*big.Rat,
	oneCap *loftCylinderOneCapData) (*Body, error) {
	// This rewrite has its own fixed work envelope. The source ring and its
	// chorded inset bound the constructed mesh; the crossing audit still
	// performs every exact contact check it would under the default ceiling.
	const (
		maxRimVertices   = 4_096
		maxInsetVertices = 4_096
		maxTriangles     = 32_768
	)
	if len(source.outer[0]) > maxRimVertices || len(source.outer[1]) > maxRimVertices {
		return nil, fmt.Errorf("%w: loft/cylinder cap outline exceeds the chamfer work ceiling", ErrUnsupported)
	}
	if len(fp.vertexBound) != len(fp.verts) || len(fp.src) != len(fp.tris) {
		return nil, fmt.Errorf("%w: loft/cylinder source has incomplete mesh proof", ErrUnsupported)
	}
	old := &joinedMesh{verts: fp.verts, tris: fp.tris, src: fp.src, groups: fp.groups}
	for cap := range 2 {
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		valid := false
		if len(source.inner[cap]) > 0 {
			valid = loftCylinderAuditAnnulus(ctx, old, source.outer[cap], source.inner[cap],
				source.capGroups[cap], level, cap == 0)
		} else {
			valid = loftCylinderAuditCap(ctx, old, source.outer[cap], source.capGroups[cap], level, cap == 0)
		}
		if !valid {
			return nil, fmt.Errorf("%w: loft/cylinder source cap no longer tiles its ring", ErrUnsupported)
		}
	}
	groups, err := loftCylinderChamferSideGroups(fp, source)
	if err != nil {
		return nil, err
	}
	mesh := &joinedMesh{verts: append([]r3.Vec(nil), fp.verts...),
		groups: append([]facetGroup(nil), fp.groups...)}
	bounds := append([]float64(nil), fp.vertexBound...)
	for i, tri := range fp.tris {
		if fp.src[i] == source.boreGroup {
			mesh.add(tri, fp.src[i])
		}
	}
	middleFirst := len(mesh.tris)
	clipped, err := loftCylinderChamferClipSide(ctx, mesh, &bounds, fp, source, groups, setback)
	if err != nil {
		return nil, err
	}
	middle := [2]int{middleFirst, len(mesh.tris)}
	if !active[1] {
		if err := loftCylinderChamferRestoreTop(mesh, fp, source, groups, clipped.rings[1]); err != nil {
			return nil, err
		}
	}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		if !loftCylinderBoreRingSimple(ctx, mesh.verts, clipped.rings[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder side level is not one simple outer ring", ErrUnsupported)
		}
	}
	var inset [2][]int
	var bands [2][2]int
	var radialReach [2]float64
	maxInset := 0.0
	for cap := range 2 {
		if !active[cap] {
			inset[cap] = source.outer[cap]
			bands[cap] = [2]int{len(mesh.tris), len(mesh.tris)}
			continue
		}
		samples, err := loftCylinderInset(ctx, fp, source.outer[cap], setback)
		if err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d inset: %w", cap, err)
		}
		if len(samples.points) > maxInsetVertices {
			return nil, fmt.Errorf("%w: loft/cylinder inset exceeds the chamfer work ceiling", ErrUnsupported)
		}
		maxInset = max(maxInset, samples.bound)
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		inset[cap] = make([]int, len(samples.points))
		for i, p := range samples.points {
			p.Z = level
			inset[cap][i] = len(mesh.verts)
			mesh.verts = append(mesh.verts, p)
			bounds = append(bounds, proofbound.AbsSumUpper(fp.meshBound, samples.bound))
		}
		if !loftCylinderBoreRingSimple(ctx, mesh.verts, inset[cap]) {
			return nil, fmt.Errorf("%w: loft/cylinder inset is not one simple outer ring", ErrUnsupported)
		}
		inSource, err := loftCylinderChamferRingNested(ctx, mesh.verts, source.outer[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if !inSource {
			return nil, fmt.Errorf("%w: loft/cylinder inset crosses its source cap", ErrUnsupported)
		}
		if active[1] {
			inSide, err := loftCylinderChamferRingNested(ctx, mesh.verts, clipped.rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !inSide {
				return nil, fmt.Errorf("%w: loft/cylinder inset crosses its side-level ring", ErrUnsupported)
			}
		}
		if err := loftCylinderChamferHeldOutlineGap(ctx, mesh.verts,
			source.outer[cap], inset[cap], fp.meshBound); err != nil {
			return nil, err
		}
		if err := loftCylinderChamferExteriorLineage(fp, source, cap); err != nil {
			return nil, err
		}
		outerRadius := proofbound.AbsSumUpper(source.blankRadius, 0.01)
		radialReach[cap], err = loftCylinderChamferCapRadialReach(source, mesh.verts,
			inset[cap], cap, outerRadius)
		if err != nil {
			return nil, err
		}
		if err := loftCylinderChamferExteriorRimInside(ctx, source, mesh.verts,
			inset[cap], cap, outerRadius); err != nil {
			return nil, err
		}
		if len(source.inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.boreRadius, fp.meshBound, samples.bound,
				clipped.clipBound)
			for i, index := range inset[cap] {
				next := inset[cap][(i+1)%len(inset[cap])]
				if loftcylinder.TriangleMinRadius(mesh.verts, [3]int{index, next, next}) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder inset edge reaches the exact bore", ErrUnsupported)
				}
			}
		}
		capGroup := source.capGroups[cap]
		role := roleCapStart
		if cap == 1 {
			role = roleCapEnd
		}
		mesh.groups[capGroup].origins = append(append([]FeatureRef(nil), mesh.groups[capGroup].origins...),
			FeatureRef{producer: ref, Role: role})
		if err := loftCylinderChamferTriangulate(ctx, mesh, inset[cap], source.inner[cap], capGroup, cap == 0); err != nil {
			return nil, fmt.Errorf("loft/cylinder cap %d: %w", cap, err)
		}
		bandGroup := len(mesh.groups)
		mesh.groups = append(mesh.groups, facetGroup{origins: []FeatureRef{{producer: ref,
			Role: fmt.Sprintf("chamferCap(%d,0,0)", cap)}}})
		bandFirst := len(mesh.tris)
		if err := loftCylinderChamferBand(mesh, clipped.rings[cap], inset[cap], samples, bandGroup); err != nil {
			return nil, fmt.Errorf("loft/cylinder band %d: %w", cap, err)
		}
		bands[cap] = [2]int{bandFirst, len(mesh.tris)}
		if len(source.inner[cap]) > 0 {
			boreReach := proofbound.AbsSumUpper(source.boreRadius, fp.meshBound, samples.bound,
				clipped.clipBound)
			for _, tri := range mesh.tris[bandFirst:] {
				if loftcylinder.TriangleMinRadius(mesh.verts, tri) <= boreReach {
					return nil, fmt.Errorf("%w: loft/cylinder cap band reaches the exact bore", ErrUnsupported)
				}
			}
		}
	}
	if err := orientJoinedMesh(mesh); err != nil {
		return nil, err
	}
	if err := tessellation.RequireVertexLinks(ctx, len(mesh.verts), mesh.tris); err != nil {
		return nil, err
	}
	ideal := loftCylinderChamferIdealVertices(mesh.verts, clipped)
	var bandSideEdges [2]map[[2]int]struct{}
	for cap := range 2 {
		if !active[cap] {
			continue
		}
		capLevel := proofarith.FloatRat(source.z0)
		sideLevel := new(big.Rat).Add(capLevel, proofarith.FloatRat(setback))
		if cap == 1 {
			capLevel = proofarith.FloatRat(source.z1)
			sideLevel = new(big.Rat).Sub(capLevel, proofarith.FloatRat(setback))
		}
		if !loftCylinderChamferExactRingSimple(ctx, ideal, clipped.rings[cap]) {
			return nil, fmt.Errorf("%w: ideal loft/cylinder side section is not simple", ErrUnsupported)
		}
		if active[1] {
			nested, err := loftCylinderChamferExactNested(ctx, ideal, clipped.rings[cap], inset[cap])
			if err != nil {
				return nil, err
			}
			if !nested {
				return nil, fmt.Errorf("%w: ideal loft/cylinder inset crosses its side section", ErrUnsupported)
			}
		}
		bandSideEdges[cap], err = loftCylinderChamferBandBoundary(mesh, bands[cap], clipped.rings[cap], inset[cap])
		if err != nil {
			return nil, err
		}
		if err := loftCylinderChamferIdealBandLevels(mesh, ideal, bands[cap], inset[cap],
			capLevel, sideLevel); err != nil {
			return nil, err
		}
	}
	if err := loftCylinderChamferContactAudit(ctx, fp, mesh, source, clipped,
		bands, bandSideEdges, ideal, setback); err != nil {
		return nil, err
	}
	if err := loftCylinderChamferIdealBandAudit(ctx, mesh, ideal, bands, clipped.clipBound); err != nil {
		return nil, err
	}
	bridgeReach := 0.0
	if active[1] {
		bridgeReach, err = loftCylinderChamferBridgeCap(ctx, fp, source, mesh, ideal,
			bands[1], inset[1], setback)
		if err != nil {
			return nil, err
		}
	}
	needed, err := loftmesh.CylinderChamferBound(fp.meshBound, maxInset, clipped.clipBound, bridgeReach)
	if err != nil {
		return nil, err
	}
	if needed > allowance || allowance > 0.1 || allowance <= 0 || proofbound.IsNonFinite(allowance) {
		return nil, fmt.Errorf("%w: cap chamfer has no finite boundary allowance", ErrUnsupported)
	}
	for cap, reach := range radialReach {
		if !active[cap] {
			continue
		}
		if cap == 1 {
			reach = proofbound.AbsSumUpper(reach, bridgeReach)
		}
		if proofbound.AbsSumUpper(reach, clipped.clipBound) >= allowance {
			return nil, fmt.Errorf("%w: cap %d radial reach exceeds the published boundary bound",
				ErrUnsupported, cap)
		}
	}
	if err := loftCylinderChamferNearBandReach(ctx, mesh, fp, source, ideal, bands,
		clipped.clipBound, allowance, !active[1], reflectedShift); err != nil {
		return nil, err
	}
	if err := loftCylinderChamferBoreInheritance(fp, mesh, source); err != nil {
		return nil, err
	}
	if err := loftCylinderChamferMiddleSideReach(ctx, fp, mesh, source, ideal,
		middle, clipped.clipBound); err != nil {
		return nil, err
	}
	// The exact band-to-source witness consumes the full certified tolerance.
	// Publish that allowance so VerifyAll does not understate the new boundary.
	bound := allowance
	if err := loftCylinderChamferIdealOrientation(ctx, mesh, ideal); err != nil {
		return nil, err
	}
	if err := loftCylinderChamferSolidWitness(ctx, fp, source, setback); err != nil {
		return nil, err
	}
	if len(mesh.tris) > maxTriangles {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer exceeds the triangle work ceiling", ErrUnsupported)
	}
	if err := loftCylinderChamferCrossingAudit(ctx, mesh, source, inset, maxTriangles); err != nil {
		return nil, err
	}
	for cap := range 2 {
		level := source.z0
		if cap == 1 {
			level = source.z1
		}
		valid := false
		if len(source.inner[cap]) > 0 {
			valid = loftCylinderAuditAnnulus(ctx, mesh, inset[cap], source.inner[cap],
				source.capGroups[cap], level, cap == 0)
		} else {
			valid = loftCylinderAuditCap(ctx, mesh, inset[cap], source.capGroups[cap], level, cap == 0)
		}
		if !valid {
			return nil, fmt.Errorf("%w: loft/cylinder inset cap does not tile its ring", ErrUnsupported)
		}
	}
	for i := range bounds {
		bounds[i] = max(bounds[i], bound)
	}
	var usedIdeal []bool
	if oneCap != nil {
		usedIdeal = make([]bool, len(ideal))
		for _, tri := range mesh.tris {
			for _, index := range tri {
				usedIdeal[index] = true
			}
		}
	}
	bounds, remap := loftCylinderChamferCompact(mesh, bounds)
	if oneCap != nil {
		oneCap.ideal = make([]proofarith.Xpt, len(mesh.verts))
		for i, x := range ideal {
			if usedIdeal[i] {
				oneCap.ideal[remap[i]] = x
			}
		}
		oneCap.band = bands[0]
	}
	roundArea, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris,
		clipped.clipBound)
	if err != nil {
		return nil, err
	}
	// The ideal rational clip applies one fixed cutter to both the source and
	// held solids. Vertex motion from that ideal shell to the stored mesh is
	// charged separately; its two embedded closed surfaces share indices.
	volumeGap := proofbound.AbsSumUpper(fp.volSymDiff,
		proofbound.SweptVolumeAllow(clipped.clipBound, roundArea))
	area, err := proofbound.PerturbedAreaUpperContext(ctx, mesh.verts, mesh.tris, bound)
	if err != nil {
		return nil, err
	}
	result := fp
	result.verts, result.tris, result.src, result.groups = mesh.verts, mesh.tris, mesh.src, mesh.groups
	result.vertexBound, result.meshBound = bounds, bound
	result.volSymDiff, result.areaSlack = volumeGap, proofbound.AbsSumUpper(fp.areaSlack, area)
	result.loftCylinderJoin, result.loftCylinderBore = nil, nil
	body, err := buildFacetedBody(ctx, d, ref, result)
	if err != nil {
		return nil, err
	}
	if oneCap != nil {
		oneCap.capGroups = source.capGroups
		oneCap.boreGroup = source.boreGroup
		for cap := range 2 {
			oneCap.outer[cap] = make([]int, len(inset[cap]))
			for i, index := range inset[cap] {
				oneCap.outer[cap][i] = remap[index]
			}
			oneCap.inner[cap] = make([]int, len(source.inner[cap]))
			for i, index := range source.inner[cap] {
				oneCap.inner[cap][i] = remap[index]
			}
		}
	}
	return body, nil
}
