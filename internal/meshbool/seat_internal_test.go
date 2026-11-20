package meshbool

import (
	"fmt"
	"math"
	"math/big"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// revolvedSegment is a revolve record about the z axis whose meridian runs from (0, 2) to (z1, rho1).
func revolvedSegment(t *testing.T, z1, rho1 float64) *surfacenormal.Revolved {
	t.Helper()
	point := func(x float64) proofbound.RatInterval {
		r := new(big.Rat).SetFloat64(x)
		return proofbound.RatInterval{Lo: r, Hi: r}
	}
	vec := func(v r3.Vec) proofbound.IvVec3 {
		iv, ok := proofbound.IvVec3Of(v)
		require.True(t, ok)
		return iv
	}
	return &surfacenormal.Revolved{
		Origin: vec(r3.Vec{}),
		Basis:  [3]proofbound.IvVec3{vec(r3.NewVec(1, 0, 0)), vec(r3.NewVec(0, 1, 0)), vec(r3.NewVec(0, 0, 1))},
		Ends:   [][2][2]proofbound.RatInterval{{{point(0), point(2)}, {point(z1), point(rho1)}}},
		Valid:  true,
	}
}

// TestDenotedTransversal reads a revolve record's rulings against a facet's plane (§16.2).
func TestDenotedTransversal(t *testing.T) {
	t.Parallel()
	// A facet of the plane z = 0.
	a, b, c := r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0)
	t.Run("a 30 degree chamfer cone leaves the plane", func(t *testing.T) {
		t.Parallel()
		require.True(t, denotedTransversal(revolvedSegment(t, 0.5, 2.866), a, b, c))
	})
	t.Run("a cylinder leaves the plane", func(t *testing.T) {
		t.Parallel()
		require.True(t, denotedTransversal(revolvedSegment(t, 3, 2), a, b, c))
	})
	// Rejected: a meridian that lies in the plane, or leans into it below seatSinMin, is no wall that leaves it.
	t.Run("an annulus in the plane refuses", func(t *testing.T) {
		t.Parallel()
		require.False(t, denotedTransversal(revolvedSegment(t, 0, 3), a, b, c))
	})
	t.Run("a cone flatter than seatSinMin refuses", func(t *testing.T) {
		t.Parallel()
		require.False(t, denotedTransversal(revolvedSegment(t, 1e-4, 3), a, b, c))
	})
	t.Run("a facet across the axis refuses", func(t *testing.T) {
		t.Parallel()
		// The plane x = 0 contains the cylinder's rulings.
		require.False(t, denotedTransversal(revolvedSegment(t, 3, 2), r3.NewVec(0, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)))
	})
}

// TestTrueFacePlane reads a placed face's exact plane from its record (§17.1).
func TestTrueFacePlane(t *testing.T) {
	t.Parallel()
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 6), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tag := surfacegeom.Plane{Frame: frame}
	t.Run("a proved placed plane", func(t *testing.T) {
		t.Parallel()
		f := &topo.Face{Surface: tag, RetainedTag: true, NormalBound: 1e-15, PlacedPlane: &tag}
		pl := trueFacePlane(f)
		require.NotNil(t, pl)
		require.True(t, pl.holds(r3.NewVec(3, -2, 6)))
		require.False(t, pl.holds(r3.NewVec(3, -2, 6.000000000000001)))
	})
	t.Run("a revolve record at one level", func(t *testing.T) {
		t.Parallel()
		r := revolvedSegment(t, 0, 3)
		pl := trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, NormalBound: 2, Denoted: r})
		require.NotNil(t, pl)
		require.True(t, pl.holds(r3.NewVec(1, 1, 0)))
	})
	// Rejected: a claim that names another tag, or a record whose meridian climbs, states no plane.
	t.Run("a stale claim", func(t *testing.T) {
		t.Parallel()
		other, err := r3.NewFrame(r3.NewVec(0, 0, 7), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		claim := surfacegeom.Plane{Frame: other}
		require.Nil(t, trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, PlacedPlane: &claim}))
	})
	t.Run("a climbing meridian", func(t *testing.T) {
		t.Parallel()
		require.Nil(t, trueFacePlane(&topo.Face{Surface: tag, RetainedTag: true, Denoted: revolvedSegment(t, 1, 3)}))
	})
}

// TestRimPointOnCarrier reads a point contact at a wall's rim vertex against a carrier-exact facet as a rim contact
// (§16.3), and nothing else.
func TestRimPointOnCarrier(t *testing.T) {
	t.Parallel()
	// Wall facet 0 has one corner, vertex 0, on the rim; facet 1 of the other mesh is carrier-exact, facet 0 not.
	wall := &BoolMesh{
		Verts:   []r3.Vec{r3.NewVec(1, 0, 0), r3.NewVec(1, 0, -3), r3.NewVec(1.1, 0.4, -3)},
		Tris:    [][3]int{{0, 1, 2}},
		RimVert: map[int]bool{0: true},
	}
	other := &BoolMesh{Carrier: []bool{false, true}}
	keyOf := func(p proof.Xpt) string { return p.Key() }
	at := func(v r3.Vec) TriContact {
		return TriContact{Kind: ContactPoint, P0: proof.XptOf(v), P1: proof.XptOf(v)}
	}
	require.True(t, rimPointOnCarrier(wall, 0, at(r3.NewVec(1, 0, 0)), other, 1, keyOf))
	// Rejected: a point off the rim, a carrier that is not exact, a contact that is not a single point.
	require.False(t, rimPointOnCarrier(wall, 0, at(r3.NewVec(1, 0, -3)), other, 1, keyOf))
	require.False(t, rimPointOnCarrier(wall, 0, at(r3.NewVec(1, 0, 0)), other, 0, keyOf))
	seg := at(r3.NewVec(1, 0, 0))
	seg.Kind = ContactSegment
	require.False(t, rimPointOnCarrier(wall, 0, seg, other, 1, keyOf))
}

// seatPrism is the prism over the convex polygon base, counter-clockwise about up, swept along up: source 0 is the
// bottom face (outward normal against up), 1 the wall and 2 the top face, each face's facets returned in mesh order.
func seatPrism(t *testing.T, base []r3.Vec, up r3.Vec) (*BoolMesh, []int, []int, []int) {
	t.Helper()
	n := len(base)
	verts := slices.Clone(base)
	for _, v := range base {
		verts = append(verts, v.Add(up))
	}
	var tris [][3]int
	var src []int
	add := func(s int, tri [3]int) {
		tris, src = append(tris, tri), append(src, s)
	}
	for i := 1; i+1 < n; i++ {
		add(0, [3]int{0, i + 1, i})
	}
	for i := range n {
		j := (i + 1) % n
		add(1, [3]int{i, j, n + j})
		add(1, [3]int{i, n + j, n + i})
	}
	for i := 1; i+1 < n; i++ {
		add(2, [3]int{n, n + i, n + i + 1})
	}
	return seatPrismMesh(t, verts, tris, src)
}

// seatPrismMesh prepares a prism's mesh as the live Boolean does, every facet carrier-exact.
func seatPrismMesh(t *testing.T, verts []r3.Vec, tris [][3]int, src []int) (*BoolMesh, []int, []int, []int) {
	t.Helper()
	bm, err := PrepContext(t.Context(), verts, tris, src)
	require.NoError(t, err)
	bm.Carrier = make([]bool, len(tris))
	var faces [3][]int
	for f, s := range src {
		bm.Carrier[f] = true
		faces[s] = append(faces[s], f)
	}
	return bm, faces[0], faces[1], faces[2]
}

// refCarrierFace is carrierFace's reference: every corner of every facet, repeats included, read by OrientSign.
func refCarrierFace(bm *BoolMesh, facets []int) bool {
	if len(facets) == 0 {
		return false
	}
	p := bm.Tris[facets[0]]
	a, b, c := bm.Verts[p[0]], bm.Verts[p[1]], bm.Verts[p[2]]
	for _, t := range facets {
		if !bm.Carrier[t] {
			return false
		}
		for _, v := range bm.Tris[t] {
			if proof.OrientSign(a, b, c, bm.Verts[v]) != 0 {
				return false
			}
		}
	}
	return true
}

// refWallLeaves is wallLeaves's reference: every corner off the rim of every facet, repeats included, read by
// OrientSign.
func refWallLeaves(bm *BoolMesh, face *topo.Face, facets []int, edges [][3]int, pf int, slack float64) bool {
	p := bm.Tris[pf]
	a, b, c := bm.Verts[p[0]], bm.Verts[p[1]], bm.Verts[p[2]]
	normal, ok := b.Sub(a).Cross(c.Sub(a)).Normalize()
	if !ok {
		return false
	}
	onRim := map[int]bool{}
	for _, e := range edges {
		t := bm.Tris[e[0]]
		onRim[t[e[1]]], onRim[t[(e[1]+1)%3]] = true, true
	}
	for _, t := range facets {
		tri := bm.Tris[t]
		if onRim[tri[0]] && onRim[tri[1]] && onRim[tri[2]] {
			return false
		}
	}
	exact := true
	for _, t := range facets {
		exact = exact && bm.isExact(t)
	}
	if exact {
		for _, t := range facets {
			for _, v := range bm.Tris[t] {
				if !onRim[v] && proof.OrientSign(a, b, c, bm.Verts[v]) >= 0 {
					return false
				}
			}
		}
		return true
	}
	if !closedLoops(bm, edges) || !wallTransversal(face, a, b, c, normal) {
		return false
	}
	beta := 0.0
	for _, t := range facets {
		beta = max(beta, bm.FacetBound[t])
	}
	for _, t := range facets {
		for _, v := range bm.Tris[t] {
			if onRim[v] {
				continue
			}
			if proof.OrientSign(a, b, c, bm.Verts[v]) >= 0 {
				return false
			}
			if !(-normal.Dot(bm.Verts[v].Sub(a)) > beta+slack) {
				return false
			}
		}
	}
	return true
}

// refSeatSkip is seatSkip's reference: every corner of every facet, repeats included, read by OrientSign.
func refSeatSkip(bmX *BoolMesh, fx []int, bmW *BoolMesh, fw []int) bool {
	if bmX.Seats == nil || bmW.Seats == nil || len(fx) == 0 || len(fw) == 0 {
		return false
	}
	for _, s := range bmW.Seats[bmW.Src[fw[0]]] {
		p := bmW.Tris[s.pf]
		a, b, c := bmW.Verts[p[0]], bmW.Verts[p[1]], bmW.Verts[p[2]]
		ok := true
		for _, x := range fx {
			if !bmX.Carrier[x] {
				ok = false
				break
			}
			for _, v := range bmX.Tris[x] {
				if proof.OrientSign(a, b, c, bmX.Verts[v]) != 0 {
					ok = false
				}
			}
			if !ok || proof.XdotSign(bmX.Norms[x], bmW.Norms[s.pf]) >= 0 {
				ok = false
				break
			}
		}
		if ok {
			return true
		}
	}
	return false
}

// TestSeatOrientationMatchesPerCorner pins carrierFace, wallLeaves (exact and held walls) and seatSkip, which test
// each distinct vertex once against a plane read from the prepared lifts and normals, to their per-corner OrientSign
// references. A wall prism W stands on the plane of its bottom face F, and a plate prism X below has its top face in
// that plane; W's wall seats on F (a top facet is listed as a first seat that fails). Each fixture runs as given and
// with each vertex of W, then of X, moved: by 1/1024 off its plane, by 2⁻⁶² (which only the exact path reads where a
// coordinate is that small), and by −0.9 and −1.25 of the sweep (a top vertex then lies nearer F than the held bound,
// or past F). The planar fixture is a hexagon in z = 6; the turned one a pentagon in x + y = 0 whose apex x = 2⁻⁶²
// makes its in-plane signs exact-path reads.
//
// Legs shown to fail: reading the probe's lift as the plane point's on the exact path, testing only the first facet
// of a carrier face, keeping ok true when a seat vertex is off its plane, and reading wallLeaves's signs against a
// wall facet instead of pf each give an answer the reference does not.
func TestSeatOrientationMatchesPerCorner(t *testing.T) {
	t.Parallel()
	tiny := 0x1p-60
	var hexagon []r3.Vec
	for k := range 6 {
		s, c := math.Sincos(float64(k) * math.Pi / 3)
		hexagon = append(hexagon, r3.NewVec(4*c, 4*s, 6))
	}
	fixtures := []struct {
		name string
		base []r3.Vec
		up   r3.Vec
	}{
		{"planar", hexagon, r3.NewVec(0, 0, 3)},
		{"turned", []r3.Vec{
			r3.NewVec(-1, 1, 0), r3.NewVec(-1, 1, 2), r3.NewVec(tiny, -tiny, 3), r3.NewVec(1, -1, 2), r3.NewVec(1, -1, 0),
		}, r3.NewVec(1, 1, 0)},
	}
	seen := map[string]map[bool]int{}
	record := func(name string, got bool) {
		if seen[name] == nil {
			seen[name] = map[bool]int{}
		}
		seen[name][got]++
	}
	uncertain := 0
	for _, fx := range fixtures {
		wall, _, _, _ := seatPrism(t, fx.base, fx.up)
		below := make([]r3.Vec, len(fx.base))
		for i, v := range fx.base {
			below[i] = v.Sub(fx.up)
		}
		plate, _, _, _ := seatPrism(t, below, fx.up)
		deltas := map[string]r3.Vec{
			"by 1/1024":   r3.NewVec(1, 1, 1).Scale(1.0 / 1024),
			"by 2^-62":    r3.NewVec(0, 0x1p-62, 0),
			"by -0.9 up":  fx.up.Scale(-0.9),
			"by -1.25 up": fx.up.Scale(-1.25),
			"by zero":     {},
		}
		check := func(name string, wVerts, xVerts []r3.Vec) {
			bmW, bottom, wf, top := seatPrismMesh(t, wVerts, wall.Tris, wall.Src)
			bmX, _, _, xTop := seatPrismMesh(t, xVerts, plate.Tris, plate.Src)
			for _, v := range xVerts {
				a, b, c := bmW.Verts[bmW.Tris[bottom[0]][0]], bmW.Verts[bmW.Tris[bottom[0]][1]], bmW.Verts[bmW.Tris[bottom[0]][2]]
				if _, certain := proof.OrientSignFloat(a, b, c, v); !certain {
					uncertain++
				}
			}
			require.Equal(t, refCarrierFace(bmW, bottom), bmW.carrierFace(bottom), "%s carrierFace", name)
			record("carrierFace", refCarrierFace(bmW, bottom))

			var edges [][3]int
			for _, f := range wf {
				for k := range 3 {
					if twin, ok := bmW.TwinFacet(f, k); ok && bmW.Src[twin] == 0 {
						edges = append(edges, [3]int{f, k, twin})
					}
				}
			}
			require.NotEmpty(t, edges)
			pf := edges[0][2]
			axis, ok := fx.up.Normalize()
			require.True(t, ok)
			face := &topo.Face{Surface: surfacegeom.Cylinder{Axis: axis}}
			slack := 64 * 0x1p-52 * 10.0
			bmW.FacetBound = make([]float64, len(bmW.Tris))
			for f := range bmW.FacetBound {
				bmW.FacetBound[f] = 0.5
			}
			want := refWallLeaves(bmW, face, wf, edges, pf, slack)
			require.Equal(t, want, bmW.wallLeaves(face, wf, edges, pf, slack), "%s held wallLeaves", name)
			record("held wallLeaves", want)
			bmW.Exact = make([]bool, len(bmW.Tris))
			for f := range bmW.Exact {
				bmW.Exact[f] = true
			}
			want = refWallLeaves(bmW, face, wf, edges, pf, slack)
			require.Equal(t, want, bmW.wallLeaves(face, wf, edges, pf, slack), "%s exact wallLeaves", name)
			record("exact wallLeaves", want)

			bmW.Seats = map[int][]rimSeat{1: {{pf: top[0]}, {pf: pf}}}
			bmX.Seats = map[int][]rimSeat{}
			want = refSeatSkip(bmX, xTop, bmW, wf)
			require.Equal(t, want, seatSkip(bmX, xTop, bmW, wf), "%s seatSkip", name)
			record("seatSkip", want)
		}
		for dn, d := range deltas {
			for v := range wall.Verts {
				ws := slices.Clone(wall.Verts)
				ws[v] = ws[v].Add(d)
				check(fmt.Sprintf("%s: wall vertex %d moved %s", fx.name, v, dn), ws, plate.Verts)
			}
			for v := range plate.Verts {
				xs := slices.Clone(plate.Verts)
				xs[v] = xs[v].Add(d)
				check(fmt.Sprintf("%s: plate vertex %d moved %s", fx.name, v, dn), wall.Verts, xs)
			}
		}
	}
	for _, name := range []string{"carrierFace", "held wallLeaves", "exact wallLeaves", "seatSkip"} {
		require.Positive(t, seen[name][true], "%s accepts some case", name)
		require.Positive(t, seen[name][false], "%s refuses some case", name)
	}
	require.Positive(t, uncertain, "some in-plane sign takes the exact path")
}
