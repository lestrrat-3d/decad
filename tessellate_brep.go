package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/units"
)

// brepWallMesh is one swept face's chording: its rim polylines at z0 and z1 as
// mesh vertex indices in walk order (closed for a whole circle), its largest
// sagitta, and the chord-versus-arc charges tessellation §5 reads.
type brepWallMesh struct {
	bottom, top []int
	sag         float64
	wallSlack   float64
	capSlack    float64
	segmentArea float64
}

// tessellateBrep meshes a brepPayload (docs/general-boolean-design.md §4.4).
// Every swept face chords its own wall once with tessellation.SampleLoop, the
// sampler a prism's walls use, and emits its rim polylines at both levels.
// Every planar face triangulates its loops with triangulate2DContext, as a
// prism cap does, over the polylines of the edges it shares, so every edge is
// chorded from one sample set and the mesh closes by construction;
// RequireClosedMesh proves it. A mesh vertex is keyed by its reference
// coordinates (brepEmbeds), which a signed permutation states exactly.
func tessellateBrep(ctx context.Context, b *Body, bp brepPayload, chord float64, verify Verification) (*Mesh, error) {
	topo, err := brepTopologyContext(ctx, bp)
	if err != nil {
		return nil, err
	}
	// The largest section displacement is reserved from the chord budget, as
	// tessellatePrism reserves its own.
	section := bp.sectionDelta()
	budget := chord
	if section > 0 {
		budget = freeform.DownRound(freeform.DownRound(chord - section))
		if budget <= 0 {
			return nil, fmt.Errorf(`%w: requested tolerance %s leaves no chord budget above the body's own section displacement %s`,
				ErrUnsupported, units.Millimeters(chord), units.Millimeters(section))
		}
	}
	byRole := map[string]*Face{}
	for _, face := range b.Faces() {
		for _, origin := range face.Origins() {
			byRole[origin.Role] = face
		}
	}
	faceOf := func(fi int) (*Face, error) {
		face := byRole[bp.faces[fi].role]
		if face == nil {
			return nil, fmt.Errorf(`%w: the body carries no face for role %q`, ErrDegenerate, bp.faces[fi].role)
		}
		return face, nil
	}

	refView := bp.refView()
	var mesh Mesh
	var store, round []float64
	var canon [][3]float64
	index := map[[3]float64]int{}
	addVertex := func(c [3]float64, sample proofbound.WalkEndBound) int {
		allow := proofbound.WalkEndBoundAllow(sample)
		if i, ok := index[c]; ok {
			store[i] = math.Max(store[i], proofbound.AbsSumUpper(allow, round[i]))
			return i
		}
		v := refView.point(c[0], c[1], c[2])
		r := exactPrismPointRound(refView, c[0], c[1], c[2], v)
		mesh.vertices = append(mesh.vertices, v)
		round = append(round, r)
		store = append(store, proofbound.AbsSumUpper(allow, r))
		canon = append(canon, c)
		index[c] = len(canon) - 1
		return len(canon) - 1
	}
	faceTrim := map[*Face]float64{}
	faceAxial := map[*Face]float64{}
	edgePoly := make([][]int, len(topo.edges))
	walls := map[int]brepWallMesh{}
	work := freeform.NewFreeformWork()
	for fi, f := range bp.faces {
		if f.planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		wm, err := brepChordWall(ctx, f, topo.walls[fi], topo.embeds[fi], face, budget, work, addVertex)
		if err != nil {
			return nil, err
		}
		walls[fi] = wm
		for _, ui := range topo.faceUses[fi] {
			switch u := topo.uses[ui]; u.Part {
			case brepRim0:
				edgePoly[topo.edgeOf[ui]] = wm.bottom
			case brepRim1:
				edgePoly[topo.edgeOf[ui]] = wm.top
			case brepSide0:
				edgePoly[topo.edgeOf[ui]] = []int{wm.bottom[0], wm.top[0]}
			case brepSide1:
				last := len(wm.bottom) - 1
				edgePoly[topo.edgeOf[ui]] = []int{wm.bottom[last], wm.top[last]}
			}
		}
		for j := 0; j+1 < len(wm.bottom); j++ {
			mesh.addTriangle([3]int{wm.bottom[j], wm.bottom[j+1], wm.top[j+1]}, face)
			mesh.addTriangle([3]int{wm.bottom[j], wm.top[j+1], wm.top[j]}, face)
		}
		faceTrim[face] = wm.sag
		faceAxial[face] = proofbound.AbsSumUpper(math.Max(f.z0Delta, f.z1Delta), f.delta)
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, wm.wallSlack)
	}
	if err := brepWallClearance(ctx, bp, topo, walls, canon); err != nil {
		return nil, err
	}

	// faceCapSlack is each planar face's own share of its curved edges'
	// chord-versus-arc area: one cap's share per curved edge it bounds.
	faceCapSlack := map[*Face]float64{}
	for fi, f := range bp.faces {
		if !f.planar() {
			continue
		}
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		e := topo.embeds[fi]
		loops := make([][]int, len(topo.planar[fi]))
		loopSag := make([]float64, len(loops))
		var pts []Point2
		var meshIdx []int
		trim, capSlack := 0.0, 0.0
		for _, ui := range topo.faceUses[fi] {
			u := topo.uses[ui]
			ei := topo.edgeOf[ui]
			owner := topo.uses[topo.edges[ei][0]]
			poly := edgePoly[ei]
			if poly == nil {
				poly = []int{addVertex(owner.DirFrom, owner.Walk.StartBound), addVertex(owner.DirTo, owner.Walk.EndBound)}
				edgePoly[ei] = poly
			}
			if !topo.forward(ui) {
				poly = slices.Clone(poly)
				slices.Reverse(poly)
			}
			if wm, ok := walls[owner.Face]; ok && brepIsRim(owner.Part) {
				trim = math.Max(trim, wm.sag)
				loopSag[u.Loop] = math.Max(loopSag[u.Loop], wm.sag)
				capSlack = proofbound.AbsSumUpper(capSlack, wm.capSlack)
			}
			for _, vi := range poly[:len(poly)-1] {
				local := e.Local(canon[vi])
				pts = append(pts, Point2{U: local[0], V: local[1]})
				meshIdx = append(meshIdx, vi)
				loops[u.Loop] = append(loops[u.Loop], len(pts)-1)
			}
		}
		if err := requireLoopClearance(ctx, pts, loops, loopSag); err != nil {
			return nil, err
		}
		tris, err := triangulate2DContext(ctx, pts, loops)
		if err != nil {
			return nil, err
		}
		for _, tri := range tris {
			a, bb, c := meshIdx[tri[0]], meshIdx[tri[1]], meshIdx[tri[2]]
			if !f.outward {
				bb, c = c, bb
			}
			mesh.addTriangle([3]int{a, bb, c}, face)
		}
		faceTrim[face] = trim
		faceAxial[face] = proofbound.AbsSumUpper(f.z0Delta, f.delta)
		faceCapSlack[face] = capSlack
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, capSlack)
	}
	if bp.xform.IsReflection() {
		for i := range mesh.triangles {
			mesh.triangles[i][1], mesh.triangles[i][2] = mesh.triangles[i][2], mesh.triangles[i][1]
		}
	}
	if err := liftTessellationError(tessellation.RequireClosedMesh(mesh.triangles)); err != nil {
		return nil, err
	}
	storeMax, err := requireDerivableStore(store)
	if err != nil {
		return nil, err
	}
	if err := composeFaceBounds(&mesh, faceTrim, faceAxial, store, 0); err != nil {
		return nil, err
	}
	if verify < VerifyAll {
		return &mesh, nil
	}

	// Occupied volume (tessellation §5's prism term applied per face): each
	// curved wall's circular-segment slivers over its height, each wall's
	// section band over its height, each planar face's level displacement over
	// its area, and every computed coordinate's swept volume. Absolute sums
	// throughout: an occupied-volume bound admits no cancellation.
	areaUpper := meshFaceAreaUpper(&mesh, store)
	var terms []float64
	for fi, f := range bp.faces {
		face, err := faceOf(fi)
		if err != nil {
			return nil, err
		}
		if f.planar() {
			region := brepPlanarDisplacement(f, topo.planar[fi])
			mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, region)
			terms = append(terms, proofbound.ProductUpper(f.z0Delta,
				proofbound.AbsSumUpper(areaUpper[face], faceCapSlack[face], region)))
			continue
		}
		w := topo.walls[fi]
		height := proofbound.UpRound(f.z1 - f.z0)
		moved := proofbound.AbsSumUpper(height, f.z0Delta, f.z1Delta)
		mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack,
			proofbound.ProductUpper(proofbound.SectionDisplacementLength(f.delta, 1), moved))
		terms = append(terms,
			proofbound.ProductUpper(height, walls[fi].segmentArea),
			proofbound.ProductUpper(moved, proofbound.SectionDisplacementArea(f.delta, 1, proofbound.AbsSumUpper(w.Length, w.LengthBound))))
	}
	mesh.areaSlack = proofbound.AbsSumUpper(mesh.areaSlack, meshStoreAreaAllow(&mesh, store))
	terms = append(terms, proofbound.SweptVolumeAllow(storeMax,
		proofbound.PerturbedAreaUpper(mesh.vertices, mesh.triangles, storeMax)))
	if err := publishSymDiff(&mesh, terms); err != nil {
		return nil, err
	}
	return &mesh, nil
}

// brepPlanarDisplacement is the area a planar face's section displacement
// moves: proofbound.SectionDisplacementArea over its own walks.
func brepPlanarDisplacement(f brepFace, walks [][]survey2d.SegmentWalk) float64 {
	if f.delta == 0 {
		return 0
	}
	count := 0
	perimeter := 0.0
	for _, loop := range walks {
		for _, w := range loop {
			perimeter = proofbound.AbsSumUpper(perimeter, w.Length, w.LengthBound)
			count++
		}
	}
	return proofbound.SectionDisplacementArea(f.delta, count, perimeter)
}

// brepChordWall chords one swept face's wall with tessellation.SampleLoop
// over its one walk and places its rim samples at both levels. An open wall's polyline ends at its walk's end; a whole
// circle's closes on its first sample.
func brepChordWall(ctx context.Context, f brepFace, w survey2d.SegmentWalk, e brepEmbed, face *Face, chord float64,
	work *freeform.FreeformWork, addVertex func([3]float64, proofbound.WalkEndBound) int) (brepWallMesh, error) {
	walk := survey2d.SideWalk{SegmentWalk: w, Segs: []int{0}}
	sampled, err := tessellation.SampleLoop[*Face]([]survey2d.SideWalk{walk}, []CurveSegment{f.wall}, chord,
		f.z1-f.z0, work, proofbound.NewWorkBudget(ctx), func(survey2d.SideWalk) (*Face, error) { return face, nil },
		chordStationBound)
	if err != nil {
		return brepWallMesh{}, err
	}
	samples, bounds := sampled.Samples, sampled.BoundOf
	if !w.Closed {
		samples = append(samples, Point2{U: w.EndU, V: w.EndV})
		bounds = append(bounds, w.EndBound)
	}
	wm := brepWallMesh{sag: sampled.MaxSag, wallSlack: sampled.WallSlack, capSlack: sampled.CapSlack,
		segmentArea: sampled.SegmentArea}
	for j, p := range samples {
		wm.bottom = append(wm.bottom, addVertex(e.Canon(p.U, p.V, f.z0), bounds[j]))
		wm.top = append(wm.top, addVertex(e.Canon(p.U, p.V, f.z1), bounds[j]))
	}
	if w.Closed {
		wm.bottom = append(wm.bottom, wm.bottom[0])
		wm.top = append(wm.top, wm.top[0])
	}
	return wm, nil
}

// brepWallClearance refuses a mesh whose chorded walls could cross where the
// analytic walls do not: every two walls sweeping along one reference axis over
// overlapping intervals, and sharing no endpoint, must clear each other by
// more than their summed sagittae plus the section's rounding floor — the
// per-slab loop clearance the stacked tessellator proves, read per wall pair.
// Each open wall polyline is walked out and back so the loop clearance reader
// measures exactly its own chords.
func brepWallClearance(ctx context.Context, bp brepPayload, topo *brepTopology, walls map[int]brepWallMesh, canon [][3]float64) error {
	type wallLine struct {
		axis   int
		lo, hi float64
		pts    []Point2
		sag    float64
	}
	var lines []wallLine
	for fi, f := range bp.faces {
		wm, ok := walls[fi]
		if !ok {
			continue
		}
		e := topo.embeds[fi]
		k := e.Axis[2]
		lo, hi := e.Sign[2]*f.z0, e.Sign[2]*f.z1
		if lo > hi {
			lo, hi = hi, lo
		}
		line := wallLine{axis: k, lo: lo, hi: hi, sag: wm.sag}
		for _, vi := range wm.bottom {
			c := canon[vi]
			var p [2]float64
			n := 0
			for i, x := range c {
				if i != k {
					p[n] = x
					n++
				}
			}
			line.pts = append(line.pts, Point2{U: p[0], V: p[1]})
		}
		lines = append(lines, line)
	}
	for i := range lines {
		for j := i + 1; j < len(lines); j++ {
			if err := ctx.Err(); err != nil {
				return err
			}
			a, b := lines[i], lines[j]
			if a.axis != b.axis || math.Max(a.lo, b.lo) >= math.Min(a.hi, b.hi) {
				continue
			}
			ends := []Point2{a.pts[0], a.pts[len(a.pts)-1]}
			if slices.Contains(ends, b.pts[0]) || slices.Contains(ends, b.pts[len(b.pts)-1]) {
				continue
			}
			pts := append(slices.Clone(a.pts), b.pts...)
			if err := requireLoopClearance(ctx, pts,
				[][]int{brepOutAndBack(0, len(a.pts)), brepOutAndBack(len(a.pts), len(b.pts))},
				[]float64{a.sag, b.sag}); err != nil {
				return err
			}
		}
	}
	return nil
}

// brepOutAndBack indexes an open polyline of n points starting at base as a
// closed loop that runs out to its end and back.
func brepOutAndBack(base, n int) []int {
	out := make([]int, 0, 2*n)
	for i := range n {
		out = append(out, base+i)
	}
	for i := n - 2; i >= 1; i-- {
		out = append(out, base+i)
	}
	return out
}
