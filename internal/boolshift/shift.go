package boolshift

import (
	"context"
	"fmt"
	"math"
	"unsafe"

	"github.com/lestrrat-3d/r3"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// PieceShift is how one piece of the second operand moves in a shifted retry: translated by Move, and then grown (or,
// when Grow is negative, shrunk) about the piece's centre so that its farthest vertex moves Grow outward. A growth opens
// contacts that no translation does, such as a tool flush with both faces of a plate, whose two faces need to move
// opposite ways.
type PieceShift struct {
	Move r3.Vec
	Grow float64
}

type UniformShrink struct {
	center r3.Vec
	move   r3.Vec
	grow   float64
	k      float64
	valid  bool
}

// Pieces partitions a mesh into its connected pieces, the sets of facets that
// share vertices, so a tool made of several separate bodies, such as the teeth of
// a gear, can have each piece moved on its own.
type Pieces struct {
	ofVertex []int
	ofFacet  []int
	count    int
}

func PiecesOf(m *topo.Mesh) Pieces {
	parent := make([]int, len(m.Vertices))
	for i := range parent {
		parent[i] = i
	}
	find := func(x int) int {
		for parent[x] != x {
			parent[x] = parent[parent[x]]
			x = parent[x]
		}
		return x
	}
	for _, t := range m.Triangles {
		for _, v := range t[1:] {
			if a, b := find(t[0]), find(v); a != b {
				parent[max(a, b)] = min(a, b)
			}
		}
	}
	id := map[int]int{}
	pc := Pieces{ofVertex: make([]int, len(m.Vertices)), ofFacet: make([]int, len(m.Triangles))}
	for i := range m.Vertices {
		root := find(i)
		c, ok := id[root]
		if !ok {
			c = len(id)
			id[root] = c
		}
		pc.ofVertex[i] = c
	}
	for f, t := range m.Triangles {
		pc.ofFacet[f] = pc.ofVertex[t[0]]
	}
	pc.count = len(id)
	return pc
}

// ShiftOperandMesh returns two copies of m with each piece moved by its own
// shift, a piece with none staying where it is. The decision copy keeps m's
// bounds, widened only by the rounding of each moved vertex (decisionMesh):
// the moved operand is the solid the Boolean decides about, so the contact
// gate reads it as it reads an unmoved one. The charged copy
// describes the moved mesh as an approximation of the true operand, which the
// composition of the result's bounds reads: the vertex and face bounds of a
// moved piece grow by that piece's displacement, the mesh bound by the largest,
// the symmetric-difference volume by what each move sweeps over its piece's
// boundary, and the area allowance by the rounding of the new coordinates. m is
// not modified. It returns ErrShiftDeclined when the moved mesh folds through
// itself.
func ShiftOperandMesh(ctx context.Context, m *topo.Mesh, pc Pieces, shifts map[int]PieceShift,
	shrink ...*UniformShrink) (decision, charged *topo.Mesh, displacementBound float64, err error) {
	var record *UniformShrink
	if len(shrink) > 0 {
		record = shrink[0]
	}
	return ShiftOperandMeshWithBudget(ctx, m, pc, shifts, record, nil)
}

func ShiftOperandMeshWithBudget(ctx context.Context, m *topo.Mesh, pc Pieces, shifts map[int]PieceShift, shrink *UniformShrink, replay *proofbound.ReplayBudget) (*topo.Mesh, *topo.Mesh, float64, error) {
	return shiftOperandMeshCore(ctx, m, pc, shifts, nil, shrink, replay)
}
func ShiftOperandMeshReplay(ctx context.Context, m *topo.Mesh, pc Pieces, shifts *ReplayShifts, shrink *UniformShrink, replay *proofbound.ReplayBudget) (*topo.Mesh, *topo.Mesh, float64, error) {
	return shiftOperandMeshCore(ctx, m, pc, nil, shifts, shrink, replay)
}
func shiftOperandMeshCore(ctx context.Context, m *topo.Mesh, pc Pieces, shifts map[int]PieceShift, ordered *ReplayShifts, shrink *UniformShrink, replay *proofbound.ReplayBudget) (decision, charged *topo.Mesh, displacementBound float64, err error) {
	count := len(shifts)
	var single PieceShift
	if ordered != nil {
		if ordered.budget != replay || len(ordered.entries) != 1 {
			return nil, nil, 0, fmt.Errorf("%w: replay shift input", decaderr.ErrUnsupported)
		}
		var ok bool
		single, ok, err = ordered.Get(ctx, 0)
		if err != nil {
			return nil, nil, 0, err
		}
		if !ok {
			return nil, nil, 0, fmt.Errorf("%w: replay shift component mismatch", decaderr.ErrUnsupported)
		}
		count = 1
	}
	eachShift := func(yield func(int, PieceShift) bool) {
		if ordered != nil {
			yield(0, single)
			return
		}
		for c, sh := range shifts {
			if !yield(c, sh) {
				return
			}
		}
	}
	lookupShift := func(c int) (PieceShift, bool, error) {
		if ordered != nil {
			return ordered.Get(ctx, c)
		}
		sh, ok := shifts[c]
		return sh, ok, nil
	}
	if replay != nil {
		if err := ctx.Err(); err != nil {
			return nil, nil, 0, err
		}
		if pc.count != 1 || count != 1 {
			return nil, nil, 0, fmt.Errorf("%w: replay shift requires one component", decaderr.ErrUnsupported)
		}
		if _, ok, e := lookupShift(0); e != nil {
			return nil, nil, 0, e
		} else if !ok {
			return nil, nil, 0, fmt.Errorf("%w: replay shift component mismatch", decaderr.ErrUnsupported)
		}
	}
	if replay != nil {
		v, f, s := uint64(len(m.Vertices)), uint64(len(m.Triangles)), uint64(count)
		if v > proofbound.MaxMultiwayRecordsPerCall || f > proofbound.MaxMultiwayRecordsPerCall || s > proofbound.MaxMultiwayRecordsPerCall || uint64(len(m.FaceBound)) > proofbound.MaxMultiwayRecordsPerCall {
			return nil, nil, 0, replay.ReserveRecords(ctx, proofbound.MaxMultiwayRecordsPerCall+1)
		}
		scan, ok := proofbound.WallCheckedMul(v, s)
		if !ok || scan > proofbound.MaxMultiwayWorkPerCall/2 {
			return nil, nil, 0, replay.SpendWork(ctx, proofbound.MaxMultiwayWorkPerCall+1)
		}
		// Two source-vertex scans per grown piece, plus all vertex, facet,
		// face and shift-map passes in the two output mesh constructions.
		if err := replay.SpendWork(ctx, 2*scan+8*v+8*f+5*uint64(len(m.FaceBound))+6*s); err != nil {
			return nil, nil, 0, err
		}
		// Vertex/exact arrays, growable facet groups and per-face maps.
		if err := replay.ReserveRecords(ctx, 10*v+4*f+5*uint64(len(m.FaceBound))+6*s+8); err != nil {
			return nil, nil, 0, err
		}
	}
	// A grown piece scales about the mean of its vertices, by the share of its farthest vertex's distance the shift asks.
	var centre map[int]r3.Vec
	var scale map[int]float64
	var oneCentre r3.Vec
	var oneScale float64
	var oneGrown bool
	if replay == nil {
		centre = make(map[int]r3.Vec, len(shifts))
		scale = make(map[int]float64, len(shifts))
	}
	getScale := func(c int) (float64, bool) {
		if replay != nil {
			return oneScale, oneGrown
		}
		v, ok := scale[c]
		return v, ok
	}
	getCentre := func(c int) r3.Vec {
		if replay != nil {
			return oneCentre
		}
		return centre[c]
	}
	for c, sh := range eachShift {
		if sh.Grow == 0 {
			continue
		}
		var sum r3.Vec
		n := 0
		for i, v := range m.Vertices {
			if pc.ofVertex[i] == c {
				sum, n = sum.Add(v), n+1
			}
		}
		if n == 0 {
			return nil, nil, 0, ErrShiftDeclined
		}
		mid := sum.Scale(1 / float64(n))
		radius := 0.0
		for i, v := range m.Vertices {
			if pc.ofVertex[i] == c {
				radius = math.Max(radius, v.Sub(mid).Len())
			}
		}
		if !(radius > 0) || !proofbound.FiniteVec(mid) {
			return nil, nil, 0, ErrShiftDeclined
		}
		k := sh.Grow / radius
		if replay != nil {
			oneCentre, oneScale, oneGrown = mid, k, true
		} else {
			centre[c], scale[c] = mid, k
		}
		if shrink != nil && pc.count == 1 && count == 1 && sh.Move == (r3.Vec{}) && sh.Grow < 0 &&
			k > -1 && !proofbound.IsNonFinite(k) {
			*shrink = UniformShrink{center: mid, move: sh.Move, grow: sh.Grow, k: k, valid: true}
		}
	}
	if replay != nil {
		if err := replay.ReserveArray(ctx, uint64(len(m.Vertices)), uint64(unsafe.Sizeof(r3.Vec{}))); err != nil {
			return nil, nil, 0, err
		}
		for range 2 {
			if err := replay.ReserveArray(ctx, uint64(len(m.Vertices)), uint64(unsafe.Sizeof(false))); err != nil {
				return nil, nil, 0, err
			}
		}
		if err := replay.ReserveArray(ctx, uint64(len(m.Vertices)), uint64(unsafe.Sizeof(proof.Xpt{}))); err != nil {
			return nil, nil, 0, err
		}
	}
	verts := make([]r3.Vec, len(m.Vertices))
	moved := make([]bool, len(m.Vertices))
	maxAbs := 0.0
	for i, v := range m.Vertices {
		c := pc.ofVertex[i]
		piece, _, e := lookupShift(c)
		if e != nil {
			return nil, nil, 0, e
		}
		sh := piece.Move
		if !proofbound.FiniteVec(sh) {
			return nil, nil, 0, ErrShiftDeclined
		}
		verts[i] = v.Add(sh)
		if k, grown := getScale(c); grown {
			verts[i] = verts[i].Add(v.Sub(getCentre(c)).Scale(k))
		}
		moved[i] = verts[i] != v
		maxAbs = math.Max(maxAbs, math.Max(math.Abs(verts[i].X), math.Max(math.Abs(verts[i].Y), math.Abs(verts[i].Z))))
	}
	// Each moved coordinate is the float sum of an exact coordinate and a
	// shift component, so it is within an ulp of the exact sum.
	round := 2 * proofbound.UlpOf(maxAbs)
	var displacement map[int]float64
	var oneDisplacement float64
	if replay == nil {
		displacement = make(map[int]float64, len(shifts))
	}
	getDisplacement := func(c int) (float64, bool) {
		if replay != nil {
			return oneDisplacement, c == 0
		}
		v, ok := displacement[c]
		return v, ok
	}
	worst := 0.0
	for c, sh := range eachShift {
		d := proofbound.AbsSumUpper(proofbound.DvLenUpper(proofbound.HeldDelta(sh.Move, r3.Vec{})), proofbound.Radius3D(round))
		d = proofbound.AbsSumUpper(d, math.Abs(sh.Grow))
		if proofbound.IsNonFinite(d) {
			return nil, nil, 0, ErrShiftDeclined
		}
		if replay != nil {
			oneDisplacement = d
		} else {
			displacement[c] = d
		}
		worst = max(worst, d)
	}

	h := meshbool.HeldRounding{
		Verts: verts, Tris: m.Triangles,
		Exact: make([]proof.Xpt, len(verts)), Moved: moved, Movable: make([]bool, len(verts)),
	}
	for i, v := range verts {
		if replay != nil {
			h.Exact[i], err = proof.XptOfBounded(v, func(n uint64) error {
				if err := replay.ReserveArray(ctx, n, 64); err != nil {
					return err
				}
				return replay.ReserveRecords(ctx, n)
			})
			if err != nil {
				return nil, nil, 0, err
			}
		} else {
			h.Exact[i] = proof.XptOf(v)
		}
	}
	var folded bool
	if replay != nil {
		folded, err = meshbool.HasHeldFoldReplay(ctx, h, replay.WorkBudget(ctx))
	} else {
		folded, err = meshbool.HasHeldFold(ctx, h)
	}
	if err != nil {
		return nil, nil, 0, err
	}
	if folded {
		return nil, nil, 0, ErrShiftDeclined
	}

	var beta []float64
	if replay != nil {
		beta, err = topobuild.VertexBoundsReplay(ctx, m, replay)
	} else {
		beta, err = topobuild.VertexBounds(m)
	}
	if err != nil {
		return nil, nil, 0, err
	}
	for i := range beta {
		d, _ := getDisplacement(pc.ofVertex[i])
		beta[i] = proofbound.AbsSumUpper(beta[i], d)
	}
	var faceBound map[*topo.Face]float64
	var fixedBound *topo.FaceBoundTable
	var trisOf map[int][][3]int
	var oneTris [][3]int
	if replay != nil {
		if err := replay.ReserveArray(ctx, uint64(len(m.Triangles)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
			return nil, nil, 0, err
		}
		oneTris = make([][3]int, 0, len(m.Triangles))
	} else {
		trisOf = make(map[int][][3]int, len(shifts))
	}
	if replay != nil {
		faceMove, err := topo.NewFaceBoundTable(ctx, m.ReplayFaceBoundCount(), replay)
		if err != nil {
			return nil, nil, 0, err
		}
		for f, t := range m.Triangles {
			c := pc.ofFacet[f]
			d, ok := getDisplacement(c)
			if !ok {
				continue
			}
			oneTris = append(oneTris, t)
			if f < len(m.Source) && m.Source[f] != nil {
				src := m.Source[f]
				prev, found, err := faceMove.Lookup(ctx, src, replay)
				if err != nil {
					return nil, nil, 0, err
				}
				if !found || d > prev {
					if err := faceMove.Set(ctx, src, d, replay); err != nil {
						return nil, nil, 0, err
					}
				}
			}
		}
		fixedBound, err = topo.NewFaceBoundTable(ctx, m.ReplayFaceBoundCount(), replay)
		if err != nil {
			return nil, nil, 0, err
		}
		err = m.EachReplayFaceBound(ctx, replay, func(face *topo.Face, bound float64) error {
			d, _, err := faceMove.Lookup(ctx, face, replay)
			if err != nil {
				return err
			}
			return fixedBound.Set(ctx, face, proofbound.AbsSumUpper(bound, d), replay)
		})
		if err != nil {
			return nil, nil, 0, err
		}
	} else {
		faceMove := make(map[*topo.Face]float64, len(m.FaceBound))
		for f, t := range m.Triangles {
			c := pc.ofFacet[f]
			d, ok := getDisplacement(c)
			if !ok {
				continue
			}
			trisOf[c] = append(trisOf[c], t)
			if f < len(m.Source) && m.Source[f] != nil {
				faceMove[m.Source[f]] = max(faceMove[m.Source[f]], d)
			}
		}
		faceBound = make(map[*topo.Face]float64, len(m.FaceBound))
		for f, b := range m.FaceBound {
			faceBound[f] = proofbound.AbsSumUpper(b, faceMove[f])
		}
	}
	volume, perimeter := 0.0, 0.0
	applyPiece := func(c int, tris [][3]int) error {
		d, _ := getDisplacement(c)
		var swept float64
		var err error
		if replay != nil {
			if err := replay.ReserveRecords(ctx, uint64(len(tris))+2*uint64(len(verts))+4); err != nil {
				return err
			}
			swept, err = proofbound.PerturbedAreaUpperReplay(ctx, replay.WorkBudget(ctx), verts, tris, d)
		} else {
			swept, err = proofbound.PerturbedAreaUpperContext(ctx, verts, tris, d)
		}
		if err != nil {
			return err
		}
		volume = proofbound.AbsSumUpper(volume, proofbound.SweptVolumeAllow(d, swept))
		for _, t := range tris {
			a, b, c := verts[t[0]], verts[t[1]], verts[t[2]]
			perimeter += b.Sub(a).Len() + c.Sub(b).Len() + a.Sub(c).Len()
		}
		return nil
	}
	if replay != nil {
		if err := applyPiece(0, oneTris); err != nil {
			return nil, nil, 0, err
		}
	} else {
		for c, tris := range trisOf {
			if err := applyPiece(c, tris); err != nil {
				return nil, nil, 0, err
			}
		}
	}
	perimeter = proofbound.UpRound(perimeter + proofbound.SumSlop(3*len(m.Triangles), perimeter))
	areaRound := proofbound.ProductUpper(proofbound.Radius3D(round), perimeter)

	decided, err := decisionMesh(ctx, m, pc, shifts, ordered, verts, centre, scale, oneCentre, oneScale, oneGrown, replay)
	if err != nil {
		return nil, nil, 0, err
	}

	next := *decided
	next.Bound = proofbound.AbsSumUpper(m.Bound, worst)
	next.CoordBound = proofbound.AbsSumUpper(m.CoordBound, worst)
	next.VertexBound = beta
	next.FaceBound = faceBound
	next.ReplayFaceBounds = fixedBound
	next.AreaSlack = proofbound.AbsSumUpper(m.AreaSlack, areaRound)
	next.VolSymDiff = proofbound.AbsSumUpper(m.VolSymDiff, volume)
	if proofbound.IsNonFinite(next.AreaSlack) || proofbound.IsNonFinite(next.VolSymDiff) {
		return nil, nil, 0, ErrShiftDeclined
	}
	return decided, &next, worst, nil
}

// decisionMesh returns the copy of m the gates decide on: m with each held
// vertex of a moved piece replaced by its float image verts[i]. The solid it
// stands for is the exact move of the true operand, each piece translated by
// its Move s and, when grown, scaled about its float centre c by its float
// factor k: x ↦ x + s + k·(x − c). A held vertex is the rounded image of that
// move, so its bound grows by the distance of verts[i] from the exact image of
// m's vertex (MoveRounding), and a grown piece's bounds also stretch by
// 1 + |k|. A vertex whose image is exact keeps its bound unchanged. A held
// vertex need not be a multiple of snapShift's grid, so a translated
// coordinate is exact by the first case of snapShift's rule only: when its
// last-place unit is no larger than the grid (|v| below about 2^28·|s|) and
// the sum does not pass the power of two above |v|. A larger coordinate, or
// a sum that crosses a power of two, generally rounds. It returns ErrShiftDeclined when a move cannot be
// bounded.
func decisionMesh(ctx context.Context, m *topo.Mesh, pc Pieces, shifts map[int]PieceShift, ordered *ReplayShifts, verts []r3.Vec, centre map[int]r3.Vec,
	scale map[int]float64, oneCentre r3.Vec, oneScale float64, oneGrown bool, replay *proofbound.ReplayBudget) (*topo.Mesh, error) {
	decided := *m
	decided.Vertices = verts
	decided.Owner = nil

	var stretchOf map[int]float64
	oneStretch := 0.0
	if replay != nil {
		if oneGrown {
			if proofbound.IsNonFinite(oneScale) {
				return nil, ErrShiftDeclined
			}
			oneStretch = proofbound.AbsSumUpper(1, oneScale)
		}
	} else {
		stretchOf = make(map[int]float64, len(scale))
	}
	for c, k := range scale {
		if proofbound.IsNonFinite(k) {
			return nil, ErrShiftDeclined
		}
		stretchOf[c] = proofbound.AbsSumUpper(1, k)
	}
	if replay != nil {
		if err := replay.ReserveArray(ctx, uint64(len(verts)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
			return nil, err
		}
	}
	add := make([]float64, len(verts))
	worst, stretchMax, changed := 0.0, oneStretch, len(stretchOf) > 0 || oneGrown
	for _, st := range stretchOf {
		stretchMax = max(stretchMax, st)
	}
	for i, v := range m.Vertices {
		c := pc.ofVertex[i]
		sh, moving := shifts[c]
		if ordered != nil {
			var e error
			sh, moving, e = ordered.Get(ctx, c)
			if e != nil {
				return nil, e
			}
		}
		if !moving {
			continue
		}
		k, grown := scale[c]
		center := centre[c]
		if replay != nil {
			k, grown, center = oneScale, oneGrown, oneCentre
		}
		var r float64
		if replay != nil {
			var err error
			r, err = replayMoveRounding(ctx, replay, v, verts[i], sh.Move, center, k, grown)
			if err != nil {
				return nil, err
			}
		} else {
			r = MoveRounding(v, verts[i], sh.Move, center, k, grown)
		}
		if proofbound.IsNonFinite(r) {
			return nil, ErrShiftDeclined
		}
		add[i] = r
		worst = max(worst, r)
		changed = changed || r > 0
	}
	if !changed {
		return &decided, nil
	}

	// A face's bound covers every corner of its facets, so it takes the
	// largest addition and stretch among them.
	widen := func(b, st, a float64) float64 {
		if st > 0 {
			b = proofbound.ProductUpper(st, b)
		}
		if a > 0 {
			b = proofbound.AbsSumUpper(b, a)
		}
		return b
	}
	if replay != nil {
		faceAdd, err := topo.NewFaceBoundTable(ctx, m.ReplayFaceBoundCount(), replay)
		if err != nil {
			return nil, err
		}
		faceStretch, err := topo.NewFaceBoundTable(ctx, m.ReplayFaceBoundCount(), replay)
		if err != nil {
			return nil, err
		}
		for f, t := range m.Triangles {
			if f >= len(m.Source) || m.Source[f] == nil {
				continue
			}
			src := m.Source[f]
			prev, _, err := faceAdd.Lookup(ctx, src, replay)
			if err != nil {
				return nil, err
			}
			for _, vi := range t {
				prev = max(prev, add[vi])
			}
			if err := faceAdd.Set(ctx, src, prev, replay); err != nil {
				return nil, err
			}
			st, grown := stretchOf[pc.ofFacet[f]]
			if replay != nil {
				st, grown = oneStretch, oneGrown
			}
			if grown {
				prev, _, err := faceStretch.Lookup(ctx, src, replay)
				if err != nil {
					return nil, err
				}
				if err := faceStretch.Set(ctx, src, max(prev, st), replay); err != nil {
					return nil, err
				}
			}
		}
		bounds, err := topo.NewFaceBoundTable(ctx, m.ReplayFaceBoundCount(), replay)
		if err != nil {
			return nil, err
		}
		err = m.EachReplayFaceBound(ctx, replay, func(face *topo.Face, b float64) error {
			a, _, err := faceAdd.Lookup(ctx, face, replay)
			if err != nil {
				return err
			}
			st, _, err := faceStretch.Lookup(ctx, face, replay)
			if err != nil {
				return err
			}
			return bounds.Set(ctx, face, widen(b, st, a), replay)
		})
		if err != nil {
			return nil, err
		}
		decided.FaceBound = nil
		decided.ReplayFaceBounds = bounds
	} else {
		faceAdd := make(map[*topo.Face]float64, len(m.FaceBound))
		faceStretch := make(map[*topo.Face]float64, len(m.FaceBound))
		for f, t := range m.Triangles {
			if f >= len(m.Source) || m.Source[f] == nil {
				continue
			}
			src := m.Source[f]
			for _, vi := range t {
				faceAdd[src] = max(faceAdd[src], add[vi])
			}
			if st, grown := stretchOf[pc.ofFacet[f]]; grown {
				faceStretch[src] = max(faceStretch[src], st)
			}
		}
		decided.FaceBound = make(map[*topo.Face]float64, len(m.FaceBound))
		for f, b := range m.FaceBound {
			decided.FaceBound[f] = widen(b, faceStretch[f], faceAdd[f])
		}
	}
	if m.VertexBound != nil {
		// A mesh with no record derives one from the widened face bounds
		// (topobuild.VertexBounds), which already cover each corner's addition.
		if replay != nil {
			if err := replay.ReserveArray(ctx, uint64(len(m.VertexBound)), uint64(unsafe.Sizeof(float64(0)))); err != nil {
				return nil, err
			}
		}
		decided.VertexBound = make([]float64, len(m.VertexBound))
		for i, b := range m.VertexBound {
			st := stretchOf[pc.ofVertex[i]]
			if replay != nil {
				st = oneStretch
			}
			decided.VertexBound[i] = widen(b, st, add[i])
		}
	}
	decided.Bound = widen(m.Bound, stretchMax, worst)
	decided.CoordBound = widen(m.CoordBound, stretchMax, worst)
	if proofbound.IsNonFinite(decided.Bound) || proofbound.IsNonFinite(decided.CoordBound) {
		return nil, ErrShiftDeclined
	}
	return &decided, nil
}

// MoveRounding bounds the distance of w, the float image of the held vertex
// v, from its exact image under a piece's move: v + s for a translation, and
// v + s + k·(v − c) for a grown piece. It is zero exactly when w is the exact
// image, and +Inf when an input is not finite.
func MoveRounding(v, w, s, c r3.Vec, k float64, grown bool) float64 {
	if !proofbound.FiniteVec(v) || !proofbound.FiniteVec(w) || !proofbound.FiniteVec(s) {
		return math.Inf(1)
	}
	if !grown {
		e := r3.Vec{X: proof.AddRoundError(v.X, s.X, w.X), Y: proof.AddRoundError(v.Y, s.Y, w.Y),
			Z: proof.AddRoundError(v.Z, s.Z, w.Z)}
		if !proofbound.FiniteVec(e) {
			return math.Inf(1)
		}
		return proofbound.DvLenUpper(proof.DyVec(e))
	}
	if !proofbound.FiniteVec(c) || proofbound.IsNonFinite(k) {
		return math.Inf(1)
	}
	dv := proof.DyVec(v)
	exact := proof.DvAdd(proof.DvAdd(dv, proof.DyVec(s)), proof.DvScale(proof.DvSub(dv, proof.DyVec(c)), proof.MustDyOf(k)))
	return proofbound.DvLenUpper(proof.DvSub(proof.DyVec(w), exact))
}
