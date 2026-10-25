package loftface

import (
	"context"
	"fmt"
	"maps"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// HeldMesh is an indexed triangle mesh with the per-triangle face id and
// per-vertex displacement bound that CapExtension reads and writes. Face ids
// are opaque to this package.
type HeldMesh struct {
	Vertices    []r3.Vec
	Triangles   [][3]int
	Face        []int
	VertexBound []float64
}

// StationLink ties a held mesh to the LoftFaces cap boundary it stands for.
// Station[v] is the index j of the cap-boundary station whose held vertex
// equals vertex v as floats, or -1 for a vertex that is no station. The cap
// ring must be exactly the station chords: ring edge (a, b) is chord j when
// {Station[a], Station[b]} is {j, j+1 mod n}. ChordBound[j] is the certified
// two-sided distance between the held chord j and the true cap-boundary piece
// it stands for, curved-profile departure included.
type StationLink struct {
	Station    []int
	ChordBound []float64
}

// Slab is what Gate E5 reads about the glued prism. Eps is the largest
// Side*phi over every vertex of the old held cap and of the held prism. Low
// and High are the exact axis box of those vertices, and LowF and HighF the
// same box as floats. ExtCap holds the extension-cap vertices.
type Slab struct {
	Eps         *big.Rat
	Low, High   RatVec
	LowF, HighF [3]float64
	ExtCap      []r3.Vec
}

// Extension is the held proxy of a LoftFaces solid with an exact prism glued to
// one end cap. It carries no volume or area charge: the proxy stands for the
// unextended operand under the clearance gate ExtensionClearsCone, which
// proves the prism changes no held contact and no retained piece.
type Extension struct {
	HeldMesh
	// FaceBound is the maximum facet bound over the final facets of every
	// face whose triangles changed: the end cap's face and each wall face
	// the cap meets. A facet's bound is the maximum of its corners' vertex
	// bounds.
	FaceBound map[int]float64
	// CoordCharge adds to the operand's coordinate bound: the largest rounding
	// gap of an extension vertex.
	CoordCharge float64
	// Slab is the clearance data of the prism.
	Slab Slab
}

// CapExtension replaces the end-cap triangles of held with the cap of a prism
// of length h glued to the cap on the side away from the loft. The prism
// direction is -plane.Side*N/|N|up, so side*phi decreases strictly along it
// and the prism lies in {side*phi <= 0}. Each ring vertex v gets the extension
// vertex round(v + h*dir), whose bound is the ring vertex bound plus the
// exact rounding gap radius. The ring edges become extension walls, each
// carrying the face id of the held wall it continues.
//
// The cap triangles are the triangles whose Face is capFace. Their boundary
// edges must form one closed cycle, and each boundary edge must meet exactly
// one non-cap triangle. Any other shape refuses. The proxy must pass the exact
// closed, oriented, embedded audit and have positive exact volume. Every
// refusal wraps decaderr.ErrUnsupported; the audit is reject-only.
//
// Each ring vertex must be a station vertex and each ring edge a station
// chord (link); any mismatch refuses. An extension wall over chord j stands
// for the translate of the true curve piece of chord j, so the bounds of its
// two extension vertices are raised to
// w_j = AbsSumUpper(max(beta(a), beta(b), chord_j), max(gap(a_e), gap(b_e))).
func CapExtension(ctx context.Context, held HeldMesh, capFace int, plane TangentPlane, h *big.Rat,
	link StationLink) (Extension, error) {
	refuse := func(why string) (Extension, error) {
		return Extension{}, fmt.Errorf("%w: the cone-tangent cap extension %s", decaderr.ErrUnsupported, why)
	}
	if plane.D == nil || plane.Side == 0 || h == nil || h.Sign() <= 0 {
		return refuse("has no usable plane or length")
	}
	nTri := len(held.Triangles)
	if len(held.Face) != nTri || len(held.VertexBound) != len(held.Vertices) || nTri == 0 {
		return refuse("is given an inconsistent mesh")
	}
	for _, v := range held.Vertices {
		if !proofbound.FiniteVec(v) {
			return refuse("is given a non-finite vertex")
		}
	}
	for _, b := range held.VertexBound {
		if proofbound.IsNonFinite(b) || b < 0 {
			return refuse("is given a missing vertex bound")
		}
	}
	capTri := make([]bool, nTri)
	capEdge := map[[2]int]bool{}
	wallEdge := map[[2]int]int{} // directed edge of a non-cap triangle -> that triangle
	for i, tri := range held.Triangles {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return Extension{}, err
			}
		}
		for _, v := range tri {
			if v < 0 || v >= len(held.Vertices) {
				return refuse("is given an out-of-range triangle")
			}
		}
		capTri[i] = held.Face[i] == capFace
		for k := range 3 {
			edge := [2]int{tri[k], tri[(k+1)%3]}
			if capTri[i] {
				if capEdge[edge] {
					return refuse("meets a cap edge used twice in one direction")
				}
				capEdge[edge] = true
				continue
			}
			if _, dup := wallEdge[edge]; dup {
				return refuse("meets a wall edge used twice in one direction")
			}
			wallEdge[edge] = i
		}
	}
	// The ring is the cap edges without a reversed partner in the cap.
	next := map[int]int{}
	wallOf := map[int]int{} // ring vertex a -> face id of the wall on edge a->b
	ringEdges := 0
	for edge := range capEdge {
		if capEdge[[2]int{edge[1], edge[0]}] {
			continue
		}
		ringEdges++
		if _, dup := next[edge[0]]; dup {
			return refuse("has a ring vertex with two outgoing boundary edges")
		}
		next[edge[0]] = edge[1]
		neighbour, ok := wallEdge[[2]int{edge[1], edge[0]}]
		if !ok {
			return refuse("has a boundary edge with no wall neighbour")
		}
		if _, direct := wallEdge[edge]; direct {
			return refuse("has a boundary edge with two non-cap neighbours")
		}
		wallOf[edge[0]] = held.Face[neighbour]
	}
	if ringEdges < 3 {
		return refuse("has no closed boundary")
	}
	var start int
	for a := range next {
		start = a
		break
	}
	ring := make([]int, 0, ringEdges)
	for a, steps := start, 0; ; steps++ {
		if steps > ringEdges {
			return refuse("has a boundary that does not close")
		}
		ring = append(ring, a)
		b, ok := next[a]
		if !ok {
			return refuse("has an open boundary")
		}
		a = b
		if a == start {
			break
		}
	}
	if len(ring) != ringEdges {
		return refuse("has a boundary of more than one cycle")
	}
	inRing := make(map[int]bool, len(ring))
	for _, a := range ring {
		inRing[a] = true
	}
	// A cap vertex off the ring may touch no wall triangle.
	capVertex := map[int]bool{}
	for i, tri := range held.Triangles {
		if capTri[i] {
			for _, v := range tri {
				capVertex[v] = true
			}
		}
	}
	for i, tri := range held.Triangles {
		if capTri[i] {
			continue
		}
		for _, v := range tri {
			if capVertex[v] && !inRing[v] {
				return refuse("has a cap vertex off the ring that a wall facet uses")
			}
		}
	}

	// Every ring edge is a distinct station chord, and every ring vertex a
	// station vertex.
	n := len(link.ChordBound)
	if n < 3 || len(link.Station) != len(held.Vertices) || len(ring) != n {
		return refuse("has a ring that is not the station ring")
	}
	chordOf := make(map[int]int, n) // ring vertex a -> chord of edge a->next[a]
	usedChord := make([]bool, n)
	for _, a := range ring {
		sa, sb := link.Station[a], link.Station[next[a]]
		if sa < 0 || sa >= n || sb < 0 || sb >= n {
			return refuse("has a ring vertex that is no station vertex")
		}
		var j int
		switch {
		case (sa+1)%n == sb:
			j = sa
		case (sb+1)%n == sa:
			j = sb
		default:
			return refuse("has a ring edge that is no station chord")
		}
		if usedChord[j] {
			return refuse("has a station chord used twice")
		}
		if proofbound.IsNonFinite(link.ChordBound[j]) || link.ChordBound[j] < 0 {
			return refuse("has no chord bound")
		}
		usedChord[j] = true
		chordOf[a] = j
	}

	// The prism direction. |N|up is a rational upper bound of |N|, so the
	// length of h*dir is at most h. Only the exact sign of phi along dir
	// matters for the proof.
	normUp := proofbound.RatSqrtUp(VDot(plane.N, plane.N))
	if proofbound.IsNonFinite(normUp) || normUp <= 0 {
		return refuse("has a degenerate plane normal")
	}
	scale := ratDiv(new(big.Rat).Mul(h, big.NewRat(int64(-plane.Side), 1)), ratF(normUp))
	disp := VScale(plane.N, scale)

	out := Extension{
		Vertices:    append([]r3.Vec(nil), held.Vertices...),
		Triangles:   make([][3]int, 0, nTri+2*len(ring)),
		Face:        make([]int, 0, nTri+2*len(ring)),
		VertexBound: append([]float64(nil), held.VertexBound...)}
	// Per-vertex gaps, so each extension vertex carries its own radius.
	gapRadius := func(v int, rounded r3.Vec) float64 {
		exact := VAdd(Vec(held.Vertices[v]), disp)
		worst := new(big.Rat)
		for k := range 3 {
			gap := ratAbs(ratSub(Vec(rounded)[k], exact[k]))
			if gap.Cmp(worst) > 0 {
				worst = gap
			}
		}
		return proofbound.Radius3D(proofbound.RatFloatUp(worst))
	}
	moved := map[int]int{} // old vertex -> its extension vertex
	radii := map[int]float64{}
	var coordCharge float64
	for _, v := range slices.Sorted(maps.Keys(capVertex)) {
		exact := VAdd(Vec(held.Vertices[v]), disp)
		rounded := RoundPoint(exact)
		if !proofbound.FiniteVec(rounded) {
			return refuse("moves a vertex out of range")
		}
		radius := gapRadius(v, rounded)
		bound := proofbound.AbsSumUpper(held.VertexBound[v], radius)
		if proofbound.IsNonFinite(bound) {
			return refuse("has an unbounded extension vertex")
		}
		coordCharge = max(coordCharge, radius)
		radii[v] = radius
		if inRing[v] {
			moved[v] = len(out.Vertices)
			out.Vertices = append(out.Vertices, rounded)
			out.VertexBound = append(out.VertexBound, bound)
		} else {
			// The vertex belongs to cap triangles alone, so it moves in place.
			moved[v] = v
			out.Vertices[v] = rounded
			out.VertexBound[v] = bound
		}
	}
	// An extension wall stands for the swept translate of its chord's true
	// curve piece, so both of its extension vertices cover that piece.
	for _, a := range ring {
		b := next[a]
		w := proofbound.AbsSumUpper(
			max(held.VertexBound[a], held.VertexBound[b], link.ChordBound[chordOf[a]]),
			max(radii[a], radii[b]))
		if proofbound.IsNonFinite(w) {
			return refuse("has an unbounded wall bound")
		}
		for _, v := range [2]int{a, b} {
			out.VertexBound[moved[v]] = max(out.VertexBound[moved[v]], w)
		}
	}

	var oldCap, extCap [][3]int
	for i, tri := range held.Triangles {
		if !capTri[i] {
			out.Triangles = append(out.Triangles, tri)
			out.Face = append(out.Face, held.Face[i])
			continue
		}
		oldCap = append(oldCap, tri)
		lifted := [3]int{moved[tri[0]], moved[tri[1]], moved[tri[2]]}
		extCap = append(extCap, lifted)
		out.Triangles = append(out.Triangles, lifted)
		out.Face = append(out.Face, capFace)
	}
	var walls [][3]int
	for _, a := range ring {
		b := next[a]
		wall := [2][3]int{{a, b, moved[b]}, {a, moved[b], moved[a]}}
		for _, tri := range wall {
			walls = append(walls, tri)
			out.Triangles = append(out.Triangles, tri)
			out.Face = append(out.Face, wallOf[a])
		}
	}
	out.CoordCharge = coordCharge
	// The face bound is the maximum facet bound over all of the face's final
	// facets. It replaces the old entry, never maxes with it.
	out.FaceBound = map[int]float64{}
	changed := map[int]bool{capFace: true}
	for _, face := range wallOf {
		changed[face] = true
	}
	for i, tri := range out.Triangles {
		face := out.Face[i]
		if !changed[face] {
			continue
		}
		for _, v := range tri {
			out.FaceBound[face] = max(out.FaceBound[face], out.VertexBound[v])
		}
	}

	if err := massmoment.AuditMesh(ctx, out.Vertices, out.Triangles, false); err != nil {
		if ctx.Err() != nil {
			return Extension{}, ctx.Err()
		}
		return refuse("is not a closed, oriented, embedded mesh")
	}
	if meshVolumeSign(out.Vertices, out.Triangles) <= 0 {
		return refuse("has no positive exact volume")
	}

	// The prism alone: the old cap reversed, the walls and the extension cap,
	// over the old cap vertices at their old positions and the extension
	// vertices. Its vertices are also the set S of Gate E5.
	capList := slices.Sorted(maps.Keys(capVertex))
	oldIndex := make(map[int]int, len(capList))
	extIndex := make(map[int]int, len(capList))
	prism := make([]r3.Vec, 0, 2*len(capList))
	for _, v := range capList {
		oldIndex[v] = len(prism)
		prism = append(prism, held.Vertices[v])
	}
	for _, v := range capList {
		extIndex[moved[v]] = len(prism)
		prism = append(prism, out.Vertices[moved[v]])
	}
	var prismTris [][3]int
	for _, tri := range oldCap {
		prismTris = append(prismTris, [3]int{oldIndex[tri[0]], oldIndex[tri[2]], oldIndex[tri[1]]})
	}
	mapPrism := func(v int) int {
		if i, ok := extIndex[v]; ok {
			return i
		}
		return oldIndex[v]
	}
	for _, tri := range append(walls, extCap...) {
		prismTris = append(prismTris, [3]int{mapPrism(tri[0]), mapPrism(tri[1]), mapPrism(tri[2])})
	}
	if err := massmoment.AuditMesh(ctx, prism, prismTris, false); err != nil {
		if ctx.Err() != nil {
			return Extension{}, ctx.Err()
		}
		return refuse("has a prism that is not a closed, oriented, embedded mesh")
	}
	if meshVolumeSign(prism, prismTris) <= 0 {
		return refuse("has a prism with no positive exact volume")
	}
	slab, err := newSlab(prism, len(capList), plane)
	if err != nil {
		return refuse(err.Error())
	}
	out.Slab = slab
	if err := ctx.Err(); err != nil {
		return Extension{}, err
	}
	return out, nil
}

// newSlab computes eps_S and the axis box over the prism vertices, whose
// first nOld entries are the old cap vertices and the rest the extension
// vertices. A non-finite vertex is an error.
func newSlab(prism []r3.Vec, nOld int, plane TangentPlane) (Slab, error) {
	var slab Slab
	for i, v := range prism {
		if !proofbound.FiniteVec(v) {
			return Slab{}, fmt.Errorf("has a non-finite prism vertex")
		}
		exact := Vec(v)
		psi := plane.Psi(exact)
		if slab.Eps == nil || psi.Cmp(slab.Eps) > 0 {
			slab.Eps = psi
		}
		coords := [3]float64{v.X, v.Y, v.Z}
		for axis := range 3 {
			if i == 0 || coords[axis] < slab.LowF[axis] {
				slab.Low[axis], slab.LowF[axis] = exact[axis], coords[axis]
			}
			if i == 0 || coords[axis] > slab.HighF[axis] {
				slab.High[axis], slab.HighF[axis] = exact[axis], coords[axis]
			}
		}
	}
	slab.ExtCap = append([]r3.Vec(nil), prism[nOld:]...)
	return slab, nil
}

// meshVolumeSign returns the sign of the exact signed volume of the mesh.
func meshVolumeSign(verts []r3.Vec, tris [][3]int) int {
	total := new(big.Rat)
	for _, tri := range tris {
		a, b, c := Vec(verts[tri[0]]), Vec(verts[tri[1]]), Vec(verts[tri[2]])
		total.Add(total, VDot(a, VCross(b, c)))
	}
	return total.Sign()
}
