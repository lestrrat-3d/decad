package spiralsector

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// CapHome records the cap vertices a temporary tooth proxy moved, with the
// position the source tooth gives each. The proxy exists only so the generic
// mesh Boolean sees the tooth's caps apart from the blank's cones; the
// published result puts the caps back on their cones (RestoreCaps).
type CapHome struct {
	moved []r3.Vec
	home  []r3.Vec
	bound []float64 // the source vertex bound at home

	// pv and pt are the proxy mesh the moves came from. The Boolean makes
	// vertices on the proxy's facets, and RestoreCaps carries them with
	// the facets.
	pv []r3.Vec
	pt [][3]int

	// caps maps each held group of a tooth cap to the held group of the blank
	// cone the cap lies on.
	caps map[int]int
}

// WithCap records the held group of a tooth cap and the held group of the
// blank cone it lies on, and returns the receiver.
func (h *CapHome) WithCap(group, cone int) *CapHome {
	if h.caps == nil {
		h.caps = make(map[int]int)
	}
	h.caps[group] = cone
	return h
}

// NewCapHome pairs proxy and home vertices index by index and keeps the pairs
// that differ. bound is the vertex bound of home, and tris are the facets of
// the proxy. The three vertex slices must have one entry per vertex.
func NewCapHome(proxy, home []r3.Vec, bound []float64, tris [][3]int) *CapHome {
	h := new(CapHome)
	if len(proxy) != len(home) || len(bound) != len(home) {
		return h
	}
	h.pv, h.pt = proxy, tris
	for i, p := range proxy {
		if p == home[i] {
			continue
		}
		h.moved = append(h.moved, p)
		h.home = append(h.home, home[i])
		h.bound = append(h.bound, bound[i])
	}
	return h
}

// Append adds the pairs of other. Both must not be used afterwards except
// through the receiver.
func (h *CapHome) Append(other *CapHome) {
	h.moved = append(h.moved, other.moved...)
	h.home = append(h.home, other.home...)
	h.bound = append(h.bound, other.bound...)
	base := len(h.pv)
	h.pv = append(h.pv, other.pv...)
	for _, t := range other.pt {
		h.pt = append(h.pt, [3]int{t[0] + base, t[1] + base, t[2] + base})
	}
	for group, cone := range other.caps {
		h.WithCap(group, cone)
	}
}

// Len is the number of moved vertices.
func (h *CapHome) Len() int { return len(h.moved) }

// Mesh is the indexed result mesh RestoreCaps reads and returns. Src names the
// source group of each facet and VertexBound the bound of each vertex.
type Mesh struct {
	Verts       []r3.Vec
	Tris        [][3]int
	Src         []int
	VertexBound []float64
}

// CapRestoration is the result mesh's vertices with the proxy cap undone, with
// what the move adds to the result's bounds. The facets are the result's own.
type CapRestoration struct {
	Verts []r3.Vec
	// VolumeCharge is added to the result's symmetric-difference volume bound.
	VolumeCharge float64
	// AreaCharge is added to the result's area slack.
	AreaCharge float64
}

// The limits below bound the work and the memory of one restoration. A
// restoration that reaches one is refused, and the caller keeps the proxy mesh.
const (
	// restoreVerts is the most vertices a restoration takes on.
	restoreVerts = 1 << 18
	// restoreAttempts is the most times a restoration starts over, each time
	// leaving the caps it could not place at their proxy positions.
	restoreAttempts = 8
	// carryCell is the edge of the cells of the vertex index, in millimetres.
	carryCell = 0.25
	// carryCells is the most cells one proxy facet may cover.
	carryCells = 1 << 12
	// carryVisits is the most vertex-in-facet tests one restoration may make.
	carryVisits = 1 << 24
	// carryPlane and carryEdge are how far a result vertex may lie off a
	// proxy facet's plane and outside its edges, in millimetres and in
	// barycentric weight, and still count as made on that facet.
	carryPlane, carryEdge = 1e-7, 1e-9
	// openStep and openReach are the step and the largest distance, in
	// millimetres, a cap slides along its cone to keep its walls facing right.
	openStep, openReach = 0.002, 0.06
	// openMargin is the clearance, in millimetres, a flattened wall keeps
	// beyond the least slide that faces it right.
	openMargin = 0.006
	// keepTurn is the least cosine between a facet's normal before and after
	// the restoration, and between a flattened wall's normal and its cone's.
	keepTurn = 0.1
	// wallAcross is the cosine below which a facet counts as standing across
	// its cone, and wallFlat the cosine above which it then counts as lying in
	// the cone.
	wallAcross, wallFlat = 0.5, 0.9
	// thinFacet is the doubled area, in square millimetres, below which a
	// facet is too thin for its facing to say anything.
	thinFacet = 2e-6
	// capStand is how far, in millimetres, a restored cap stands above its blank
	// cone. The cap and the blank are then never exactly coplanar, which keeps the
	// facets between them from degenerating under the rounding of a later placement.
	capStand = 0.003
	// capStandMax is the stand beyond which a cap that still fails a check stays
	// at its proxy positions; each failed attempt doubles a cap's stand.
	capStandMax = 0.025
	// fragile is how far, in millimetres, the check for folds nudges every vertex
	// to find the pairs a placement's rounding would fold.
	fragile = 1e-11
)

// RestoreCaps returns the positions of the result's vertices with the proxy
// cap lowered onto its cone, so the tooth's caps are flush with the cones that
// trimmed them. The facets are the result's own and do not change.
//
// The proxy lifted each cap vertex off its cone by a shift whose component
// along the cone's normal is the lift. Every moved vertex gives up that
// component and keeps the rest, which slides it along the cone. Each cap, as a
// whole, then takes the mean level of the blank cone it lies on plus capStand,
// one normal shift per cap. The cap lies in its cone, within capStand, and
// keeps the layout along the cone that the Boolean proved embedded. A vertex
// the Boolean made on a facet of the proxy, and that no blank facet holds,
// moves with that facet at its barycentric weights. A vertex a blank facet
// holds stays.
//
// The walls the lift left between the cap and the blank flatten toward the
// cone, and the cap slides out along its cone, as one piece, by the least
// distance that leaves every flattened wall facing along the cone's outward
// normal, so a wall keeps a width. capStand keeps the cap and the blank from
// being exactly coplanar, so no facet between them degenerates.
//
// The restored mesh is then checked as the Boolean's result was: no pair of
// facets meets other than as neighbours, read as it is and nudged either way by
// fragile; and a placement probe (placementProbes) leaves it embedded after the
// float-box repair a Transform runs. A cap that fails a check, or that no slide
// serves, or one holding a vertex whose bound is below the bound of its source
// position, stays at its proxy positions while the other caps are restored. It
// answers false when nothing can be restored, when two moved vertices share a
// position, when a moved vertex lies on a facet of the blank or when a limit is
// reached. The caller then keeps the proxy-position mesh, which its audit
// already certified.
//
// A restored vertex lies within capStand of its cone, or on a facet of the
// proxy that moved with it, within the shift of the position the proxy bound
// already covers, so the vertex and facet bounds stand. The restored mesh
// differs from the held one only on the facets that touch a changed vertex,
// where every vertex moved by at most the largest distance. The volume between
// the two meshes is that distance times the area along the straight path, and
// the area slack adds both meshes' changed-facet areas.
func RestoreCaps(ctx context.Context, home *CapHome, m Mesh, blankFaces int) (CapRestoration, bool, error) {
	n := len(m.Verts)
	if home == nil || len(home.moved) == 0 || len(home.caps) == 0 || len(m.VertexBound) != n ||
		len(m.Src) != len(m.Tris) || n > restoreVerts {
		return CapRestoration{}, false, nil
	}
	slot := make(map[r3.Vec]int, len(home.moved))
	for i, p := range home.moved {
		if _, dup := slot[p]; dup {
			return CapRestoration{}, false, nil
		}
		slot[p] = i
	}
	skip := make([]bool, n)
	stands := make(map[int]float64)
	for range restoreAttempts {
		st := &restoreState{m: m, home: home, blankFaces: blankFaces, slot: slot, skip: skip, stands: stands}
		out, ok, err := st.run(ctx)
		if err != nil || ok {
			return out, ok, err
		}
		if len(st.pinned) > 0 {
			for _, v := range st.pinned {
				skip[v] = true
			}
			continue
		}
		if !st.leave() {
			return CapRestoration{}, false, nil
		}
	}
	return CapRestoration{}, false, nil
}

// restoreState is what one attempt at RestoreCaps works out about the result
// mesh.
type restoreState struct {
	m          Mesh
	home       *CapHome
	blankFaces int
	slot       map[r3.Vec]int
	skip       []bool          // per vertex: a cap the attempt leaves at its proxy position
	stands     map[int]float64 // per cap group: how far it stands above its cone, when not capStand

	moved    []int              // per vertex: index into home, or -1
	vertex   []int              // per home slot: the vertex, or -1
	onBlank  []bool             // per vertex: a blank facet holds it
	group    []int              // per moved vertex: its cap group, plus one
	piece    []int              // per moved vertex: the first vertex of its cap
	cone     map[int][2]float64 // per blank cone group: the summed axial and radial parts of its facets' normals
	band     map[int][]int      // per blank cone group: its vertices
	incident [][]int            // per vertex: the facets that hold it
	links    []carried          // the vertices made on facets of the proxy
	normal   []r3.Vec           // per moved vertex: the outward normal of its cone
	out      []r3.Vec           // per moved vertex: the unit direction along the cone, away from the axis
	next     []r3.Vec           // the restored position of every vertex
	byslot   []r3.Vec           // per home slot: the displacement of its vertex
	blamed   map[int]struct{}   // the caps (by first vertex) that failed this attempt
	pinned   []int              // moved vertices of folded facets, to stay at their proxy positions
}

// run makes one attempt.
func (st *restoreState) run(ctx context.Context) (CapRestoration, bool, error) {
	if ok, err := st.classify(ctx); err != nil || !ok {
		return CapRestoration{}, false, err
	}
	if ok, err := st.link(ctx); err != nil || !ok {
		return CapRestoration{}, false, err
	}
	if !st.lower() {
		return CapRestoration{}, false, nil
	}
	if ok, err := st.open(ctx); err != nil || !ok {
		return CapRestoration{}, false, err
	}
	return st.finish(ctx)
}

// leave raises the stand of each cap that failed this attempt, which moves it
// toward the proxy position it was lifted to, and marks a cap that already
// stands at capStandMax to stay at its proxy positions. It answers whether any
// cap changed.
func (st *restoreState) leave() bool {
	found := false
	raised := make(map[int]struct{})
	for i := range st.m.Verts {
		if st.moved == nil || st.moved[i] < 0 {
			continue
		}
		if _, bad := st.blamed[st.piece[i]]; !bad || st.skip[i] {
			continue
		}
		g := st.group[i] - 1
		if _, done := raised[g]; !done {
			raised[g] = struct{}{}
			if stand := st.standOf(g); stand < capStandMax {
				st.stands[g] = 2 * stand
			}
		}
		if st.standOf(g) >= capStandMax {
			st.skip[i] = true
		}
		found = true
	}
	return found
}

// standOf is how far cap group g stands above its cone.
func (st *restoreState) standOf(g int) float64 {
	if stand, ok := st.stands[g]; ok {
		return stand
	}
	return capStand
}

// blame marks the caps that hold any of the vertices as failed.
func (st *restoreState) blame(vertices ...int) {
	for _, v := range vertices {
		if st.moved[v] < 0 {
			continue
		}
		if st.blamed == nil {
			st.blamed = make(map[int]struct{})
		}
		st.blamed[st.piece[v]] = struct{}{}
	}
}

// classify finds the moved vertices, the vertices the blank holds, the cap each
// moved vertex belongs to and the cone each cap lies on.
func (st *restoreState) classify(ctx context.Context) (bool, error) {
	m := st.m
	n := len(m.Verts)
	st.moved = make([]int, n)
	st.vertex = make([]int, len(st.home.moved))
	for j := range st.vertex {
		st.vertex[j] = -1
	}
	st.onBlank = make([]bool, n)
	st.group = make([]int, n)
	st.piece = make([]int, n)
	st.incident = make([][]int, n)
	st.cone = make(map[int][2]float64, len(st.home.caps))
	st.band = make(map[int][]int)
	isBand := make(map[int]struct{}, len(st.home.caps))
	for _, cone := range st.home.caps {
		isBand[cone] = struct{}{}
	}
	seenBand := make(map[[2]int]struct{})
	var unsound []int // moved vertices whose source position cannot be taken
	for i, v := range m.Verts {
		st.moved[i], st.piece[i] = -1, i
		j, ok := st.slot[v]
		if !ok || st.skip[i] {
			continue
		}
		st.moved[i], st.vertex[j] = j, i
		if !proofbound.FiniteVec(st.home.home[j]) || m.VertexBound[i] < st.home.bound[j] {
			unsound = append(unsound, i)
		}
	}
	for f, t := range m.Tris {
		if f%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		_, isCap := st.home.caps[m.Src[f]]
		if _, ok := isBand[m.Src[f]]; ok {
			a, b, c := m.Verts[t[0]], m.Verts[t[1]], m.Verts[t[2]]
			st.coneFacet(f, b.Sub(a).Cross(c.Sub(a)), a.Add(b).Add(c).Scale(1.0/3))
		}
		if isCap {
			st.join(t[0], t[1])
			st.join(t[1], t[2])
		}
		for _, v := range t {
			st.incident[v] = append(st.incident[v], f)
			if m.Src[f] < st.blankFaces {
				st.onBlank[v] = true
			}
			if _, ok := isBand[m.Src[f]]; ok {
				if _, dup := seenBand[[2]int{m.Src[f], v}]; !dup {
					seenBand[[2]int{m.Src[f], v}] = struct{}{}
					st.band[m.Src[f]] = append(st.band[m.Src[f]], v)
				}
			}
			if isCap && st.moved[v] >= 0 {
				if st.group[v] != 0 && st.group[v] != m.Src[f]+1 {
					return false, nil
				}
				st.group[v] = m.Src[f] + 1
			}
		}
	}
	for i := range m.Verts {
		if st.moved[i] >= 0 {
			st.piece[i] = st.root(i)
			if st.onBlank[i] {
				return false, nil
			}
		}
	}
	st.fitCones()
	// A cap holding a vertex whose bound is below the bound of its source
	// position, or whose source position is not finite, stays at the proxy.
	st.blame(unsound...)
	return len(unsound) == 0, nil
}

// fitCones replaces each blank cone's summed facet normals by its normal in
// the cone's frame, from the slope of its vertices: they lie on the cone, so
// the axial position falls by k per unit of distance from the axis, and the
// normal is (1, k) along the axis and away from it, signed as the facets'.
func (st *restoreState) fitCones() {
	for g, verts := range st.band {
		var sx, sr, sxx, sxr float64
		for _, v := range verts {
			p := st.m.Verts[v]
			r := math.Hypot(p.Y, p.Z)
			sx, sr, sxx, sxr = sx+r, sr+p.X, sxx+r*r, sxr+r*p.X
		}
		n := float64(len(verts))
		den := n*sxx - sx*sx
		if den == 0 {
			continue
		}
		// x = x0 - k*rho, so k = -(n*sum(rho*x) - sum(rho)*sum(x)) / den.
		k := -(n*sxr - sx*sr) / den
		sign := 1.0
		if st.cone[g][0] < 0 {
			sign = -1
		}
		norm := math.Hypot(1, k)
		st.cone[g] = [2]float64{sign / norm, sign * k / norm}
	}
}

// root is the first vertex of the cap that holds vertex i, found through the
// joins made so far.
func (st *restoreState) root(i int) int {
	for st.piece[i] != i {
		st.piece[i] = st.piece[st.piece[i]]
		i = st.piece[i]
	}
	return i
}

// join puts the caps of vertices a and b together.
func (st *restoreState) join(a, b int) {
	if ra, rb := st.root(a), st.root(b); ra != rb {
		st.piece[rb] = ra
	}
}

// coneFacet adds one facet of a blank cone to its normal, in the cone's own
// frame: the normal's component along the axis and its component along the
// direction away from the axis at the facet. A cone's facets give its normal
// better than a cap's, which are few and small.
func (st *restoreState) coneFacet(f int, normal, centre r3.Vec) {
	radial := r3.Vec{Y: centre.Y, Z: centre.Z}
	if l := radial.Dot(radial); l > 0 {
		radial = radial.Scale(1 / math.Sqrt(l))
	}
	g := st.cone[st.m.Src[f]]
	g[0] += normal.X
	g[1] += normal.Dot(radial)
	st.cone[st.m.Src[f]] = g
}

// lower drops every moved vertex onto its cone: it keeps the part of the
// proxy shift that lies along the cone. Each cap then takes its blank cone's
// level.
func (st *restoreState) lower() bool {
	m := st.m
	st.next = append([]r3.Vec(nil), m.Verts...)
	st.normal = make([]r3.Vec, len(m.Verts))
	st.out = make([]r3.Vec, len(m.Verts))
	for i, v := range m.Verts {
		j := st.moved[i]
		if j < 0 {
			continue
		}
		if st.group[i] == 0 {
			return false
		}
		axial, radial := st.cone[st.home.caps[st.group[i]-1]][0], st.cone[st.home.caps[st.group[i]-1]][1]
		at := st.home.home[j]
		away := r3.Vec{Y: at.Y, Z: at.Z}
		if away.Dot(away) == 0 {
			return false
		}
		away = away.Scale(1 / math.Sqrt(away.Dot(away)))
		normal := r3.Vec{X: axial}.Add(away.Scale(radial))
		if normal.Dot(normal) == 0 {
			return false
		}
		normal = normal.Scale(1 / math.Sqrt(normal.Dot(normal)))
		along := away.Sub(normal.Scale(away.Dot(normal)))
		if along.Dot(along) < 1e-12 {
			return false
		}
		to := v.Sub(normal.Scale(v.Sub(at).Dot(normal)))
		if !proofbound.FiniteVec(to) {
			return false
		}
		st.normal[i], st.out[i], st.next[i] = normal, along.Scale(1/math.Sqrt(along.Dot(along))), to
	}
	return st.level()
}

// level moves each cap, as a whole, along its cone's normal until its mean
// level is the blank cone's: the cap lies in the plane of the cone it was
// trimmed by, which the blank's facets, not the tooth's, give. One shift per
// cap leaves the cap's own facets as they are.
func (st *restoreState) level() bool {
	type sum struct {
		have float64
		n    int
	}
	have := make(map[int]*sum, len(st.home.caps))
	for i := range st.m.Verts {
		if st.moved[i] < 0 {
			continue
		}
		g := st.group[i] - 1
		s := have[g]
		if s == nil {
			s = new(sum)
			have[g] = s
		}
		s.have += st.levelOf(st.next[i], g)
		s.n++
	}
	shift := make(map[int]float64, len(have))
	for g, s := range have {
		verts := st.band[st.home.caps[g]]
		if len(verts) == 0 {
			return false
		}
		want := 0.0
		for _, v := range verts {
			want += st.levelOf(st.m.Verts[v], g)
		}
		shift[g] = want/float64(len(verts)) - s.have/float64(s.n) + st.standOf(g)
	}
	for i := range st.m.Verts {
		if st.moved[i] < 0 {
			continue
		}
		st.next[i] = st.next[i].Add(st.normal[i].Scale(shift[st.group[i]-1]))
		if !proofbound.FiniteVec(st.next[i]) {
			return false
		}
	}
	return true
}

// levelOf is the level of p on the cone of cap group g: a cone is a level set
// of the part of a position along its normal.
func (st *restoreState) levelOf(p r3.Vec, g int) float64 {
	axial, radial := st.cone[st.home.caps[g]][0], st.cone[st.home.caps[g]][1]
	return (axial*p.X + radial*math.Hypot(p.Y, p.Z)) / math.Hypot(axial, radial)
}
