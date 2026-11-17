package retainedblend

import (
	"math"
	"testing"

	clr "github.com/lestrrat-3d/decad/internal/clearance"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// obliquePlane is a plane that states the departure and extent of an oblique swept wall
// (charged-window-chamfer-design.md §2).
func obliquePlane(t *testing.T, u, w r3.Vec, departure, extent float64) *topo.Face {
	t.Helper()
	f := chargedPlane(t, r3.Vec{}, u, w, 0)
	f.TagDeparture, f.NormalBound, f.ExtentDelta = departure, departure, extent
	return f
}

func TestReadChargedChainFace(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)

	t.Run("an exact face reads with no charge", func(t *testing.T) {
		t.Parallel()
		for _, f := range []*topo.Face{chargedPlane(t, r3.Vec{}, x, y, 0), chargedCylinder(3, 0)} {
			w, ok := readChargedChainFace(f)
			require.True(t, ok)
			require.Zero(t, w.delta)
			require.Zero(t, w.departure)
		}
	})

	t.Run("a cylinder or a plane that states a displacement reads with that charge", func(t *testing.T) {
		t.Parallel()
		for _, f := range []*topo.Face{chargedCylinder(3, 1e-9), chargedPlane(t, r3.Vec{}, x, y, 1e-9)} {
			w, ok := readChargedChainFace(f)
			require.True(t, ok)
			require.Equal(t, 1e-9, w.delta)
			require.Zero(t, w.departure)
			_, strict := readChainFace(f, tagExact)
			require.False(t, strict, "the exact reader still declines it")
		}
	})

	t.Run("an oblique plane reads with its extent as the charge", func(t *testing.T) {
		t.Parallel()
		f := obliquePlane(t, z, x, 1e-6, 2e-9)
		w, ok := readChargedChainFace(f)
		require.True(t, ok)
		require.Equal(t, 2e-9, w.delta)
		require.Equal(t, 1e-6, w.departure)
		for _, mode := range []tagMode{tagExact} {
			_, ok := readChainFace(f, mode)
			require.False(t, ok, "mode %d", mode)
		}
		p, ok := chainPlane(f)
		require.True(t, ok)
		require.Equal(t, 2e-9, p.delta)
		_, ok = retainedPlane(f)
		require.False(t, ok, "every other plane reader declines an oblique wall")
	})

	t.Run("a face with a charge no bound can cover is not read", func(t *testing.T) {
		t.Parallel()
		for _, bad := range []float64{-1e-9, math.NaN(), math.Inf(1)} {
			_, ok := readChargedChainFace(chargedCylinder(3, bad))
			require.False(t, ok, "cylinder %v", bad)
			_, ok = readChargedChainFace(chargedPlane(t, r3.Vec{}, x, y, bad))
			require.False(t, ok, "plane %v", bad)
			_, ok = readChargedChainFace(obliquePlane(t, z, x, 1e-6, bad))
			require.False(t, ok, "extent %v", bad)
		}
	})

	t.Run("an oblique plane is read only with every condition of its record", func(t *testing.T) {
		t.Parallel()
		cases := map[string]func(f *topo.Face){
			"no extent":                func(f *topo.Face) { f.ExtentDelta = 0 },
			"a normal bound beyond it": func(f *topo.Face) { f.NormalBound *= 2 },
			"a denoted patch":          func(f *topo.Face) { f.Denoted = nil; f.HasAxialDelta, f.AxialDelta = true, 1e-9 },
			"not retained":             func(f *topo.Face) { f.RetainedTag = false },
			"a negative departure":     func(f *topo.Face) { f.TagDeparture, f.NormalBound = -1e-6, -1e-6 },
			"an infinite departure":    func(f *topo.Face) { f.TagDeparture, f.NormalBound = math.Inf(1), math.Inf(1) },
		}
		for name, mutate := range cases {
			f := obliquePlane(t, z, x, 1e-6, 2e-9)
			_, ok := readChargedChainFace(f)
			require.True(t, ok)
			mutate(f)
			_, ok = readChargedChainFace(f)
			require.False(t, ok, name)
		}
	})

	t.Run("a stated extent with no departure is not read", func(t *testing.T) {
		t.Parallel()
		f := chargedPlane(t, r3.Vec{}, x, y, 0)
		f.ExtentDelta = 1e-9
		_, ok := readChargedChainFace(f)
		require.False(t, ok)
		c := chargedCylinder(3, 1e-9)
		c.ExtentDelta = 1e-9
		_, ok = readChargedChainFace(c)
		require.False(t, ok)
	})

	t.Run("a cylinder with a departure is not read", func(t *testing.T) {
		t.Parallel()
		c := chargedCylinder(3, 1e-9)
		c.TagDeparture = 1e-6
		_, ok := readChargedChainFace(c)
		require.False(t, ok)
	})
}

func TestChainCandidateAndCharged(t *testing.T) {
	t.Parallel()
	line, arc, _ := chargedJunction(t, 1e-9)
	require.False(t, looksLikeArc(arc) && !wallRecord(arc.Faces[1]))
	require.True(t, chainCandidate(line))
	require.True(t, chainCandidate(arc))
	require.True(t, chainCharged([]*topo.Edge{arc}))
	require.False(t, chainCharged([]*topo.Edge{line}))

	t.Run("an oblique side is a candidate no per-edge route reads", func(t *testing.T) {
		t.Parallel()
		line, _, _ := chargedJunction(t, 0)
		line.Faces[1] = obliquePlane(t, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), 1e-6, 1e-9)
		require.False(t, looksLikeLine(line))
		require.True(t, chainCandidate(line))
		require.True(t, chainCharged([]*topo.Edge{line}))
	})

	t.Run("a face the chain reader declines makes no candidate", func(t *testing.T) {
		t.Parallel()
		line, _, _ := chargedJunction(t, 0)
		line.Faces[1].TagDeparture = 1e-6
		require.False(t, chainCandidate(line))
	})
}

func TestRefuseChargedChain(t *testing.T) {
	t.Parallel()
	line, arc, _ := chargedJunction(t, 1e-9)
	closed := chainCycle{edges: []*topo.Edge{line, arc}}
	open := chainCycle{edges: []*topo.Edge{line, arc}, open: true}

	require.NoError(t, refuseChargedChain(closed, false), "a closed chamfer chain builds")
	err := refuseChargedChain(open, false)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "an open tangent chain on walls that state a displacement is not built")
	err = refuseChargedChain(open, true)
	require.ErrorContains(t, err, "an open tangent chain on walls that state a displacement is not built")
	err = refuseChargedChain(closed, true)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	require.ErrorContains(t, err, "a tangent chain fillet on walls that state a displacement is not built")

	plain, plainArc, _ := chargedJunction(t, 0)
	for _, fillet := range []bool{false, true} {
		require.NoError(t, refuseChargedChain(chainCycle{edges: []*topo.Edge{plain, plainArc}}, fillet))
		require.NoError(t, refuseChargedChain(chainCycle{edges: []*topo.Edge{plain, plainArc}, open: true}, fillet))
	}
}

func TestCheckRestApart(t *testing.T) {
	t.Parallel()
	a, b, c, far1, far2 := &topo.Vertex{}, &topo.Vertex{}, &topo.Vertex{}, &topo.Vertex{}, &topo.Vertex{}
	chain := chainCycle{edges: []*topo.Edge{{Start: a, End: b}, {Start: b, End: c}, {Start: c, End: a}}}
	apart := &topo.Edge{Start: far1, End: far2}
	touching := &topo.Edge{Start: far1, End: b}

	require.NoError(t, checkRestApart([]chainCycle{chain}, nil, false))
	require.NoError(t, checkRestApart([]chainCycle{chain}, []*topo.Edge{apart}, false), "an edge apart from the chain")
	for name, c := range map[string]struct {
		chains []chainCycle
		rest   []*topo.Edge
		fillet bool
	}{
		"an edge that shares a vertex":      {[]chainCycle{chain}, []*topo.Edge{apart, touching}, false},
		"an edge that ends at a vertex":     {[]chainCycle{chain}, []*topo.Edge{{Start: a, End: far1}}, false},
		"a fillet":                          {[]chainCycle{chain}, []*topo.Edge{apart}, true},
		"an open chain":                     {[]chainCycle{{edges: chain.edges, open: true}}, []*topo.Edge{apart}, false},
		"a closed chain beside an open one": {[]chainCycle{chain, {edges: chain.edges, open: true}}, []*topo.Edge{apart}, false},
	} {
		err := checkRestApart(c.chains, c.rest, c.fillet)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, "a selection that holds a tangent chain names only chain edges", name)
	}
}

func TestHeldCharge(t *testing.T) {
	t.Parallel()
	x, y, z := r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1)
	const delta = 1e-9
	want := planeAmp * delta

	for name, f := range map[string]*topo.Face{
		"a displaced plane":  chargedPlane(t, r3.Vec{}, x, y, delta),
		"a charged cylinder": chargedCylinder(3, delta),
		"an oblique plane":   obliquePlane(t, z, x, 1e-6, delta),
	} {
		got, err := heldCharge(0, f)
		require.NoError(t, err, name)
		require.GreaterOrEqual(t, got, want, name)
		require.InDelta(t, want, got, 1e-6*want, name)
	}
	got, err := heldCharge(0, chargedPlane(t, r3.Vec{}, x, y, 0))
	require.NoError(t, err)
	require.Zero(t, got)
	got, err = heldCharge(0, chargedCylinder(3, 0))
	require.NoError(t, err)
	require.Zero(t, got)

	t.Run("the sum is the sum of the faces", func(t *testing.T) {
		t.Parallel()
		sum := 0.0
		var err error
		for _, f := range []*topo.Face{chargedCylinder(3, delta), obliquePlane(t, z, x, 1e-6, delta),
			chargedPlane(t, r3.Vec{}, x, y, delta)} {
			sum, err = heldCharge(sum, f)
			require.NoError(t, err)
		}
		require.GreaterOrEqual(t, sum, 3*want)
		require.InDelta(t, 3*want, sum, 1e-6*want)
	})

	t.Run("an unusable displacement is refused and an unusable extent states nothing", func(t *testing.T) {
		t.Parallel()
		_, err := heldCharge(0, chargedCylinder(3, -1))
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		f := obliquePlane(t, z, x, 1e-6, math.NaN())
		got, err := heldCharge(0, f)
		require.NoError(t, err)
		require.Zero(t, got)
	})
}

// chargeFixture is a closed three-edge chain's charges: a cylinder wall of radius 3 and a plane wall.
func chargeFixture(t *testing.T, wallDelta float64, oblique bool) (chainCycle, *topo.Face, facetedrecord.Payload, []int, []r3.Vec) {
	t.Helper()
	base := chargedPlane(t, r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0), 0)
	front := chargedPlane(t, r3.Vec{}, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), 0)
	if oblique {
		front = obliquePlane(t, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), 4e-6, 1e-9)
	}
	cyl := chargedCylinder(3, wallDelta)
	other := chargedCylinder(3, wallDelta)
	other.Surface = surfacegeom.Cylinder{Origin: r3.NewVec(3, 3, 0), Axis: r3.NewVec(0, 0, 1), Radius: cyl.Surface.(surfacegeom.Cylinder).Radius}
	cyc := chainCycle{
		edges: []*topo.Edge{{Faces: []*topo.Face{base, front}}, {Faces: []*topo.Face{base, cyl}}},
		joins: []chainLink{
			{j: junction{base: base, wall1: front, wall2: cyl}},
			{j: junction{base: base, wall1: cyl, wall2: front}},
		},
	}
	fp := facetedrecord.Payload{
		Verts:       []r3.Vec{r3.NewVec(3, 0, 20), r3.NewVec(0, 3, 20)},
		VertexBound: []float64{2e-3, 3e-3},
	}
	// The held vertices lie 1e-4 off the tags' junction points.
	return cyc, base, fp, []int{0, 1}, []r3.Vec{r3.NewVec(3, 1e-4, 20), r3.NewVec(0, 3, 20)}
}

func TestNewChainCharges(t *testing.T) {
	t.Parallel()

	t.Run("a chain that reads no charge adds nothing", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 0, false)
		cc, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.NoError(t, err)
		require.Zero(t, cc)
		require.Zero(t, cc.sigma())
		w := &work{beta: []float64{1}}
		cc.raise(w, 0, 0)
		require.Equal(t, []float64{1}, w.beta)
	})

	t.Run("a charged cylinder gives each junction its shift and the chain its crease", func(t *testing.T) {
		t.Parallel()
		const delta = 1e-9
		cyc, base, fp, hv, jp := chargeFixture(t, delta, false)
		cc, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.NoError(t, err)
		require.True(t, cc.charged)
		require.Equal(t, delta, cc.delta)
		require.Zero(t, cc.deltaP)
		// e_J = β(v) + |v − V|.
		require.GreaterOrEqual(t, cc.eJ[0], 2e-3+1e-4)
		require.InDelta(t, 2e-3+1e-4, cc.eJ[0], 1e-12)
		require.GreaterOrEqual(t, cc.eJ[1], 3e-3)
		require.InDelta(t, 3e-3, cc.eJ[1], 1e-12)
		// ψ_max = (e_J + 2δ)/R + ψ_0 over the cylinder wall, a plane adding none; c_J = t·ψ²/4 at the larger e_J.
		psi := (cc.eJ[1]+2*delta)/3 + math.Sqrt(2*64*ulp*3/3)
		require.GreaterOrEqual(t, cc.crease, 0.5*psi*psi/4)
		require.InDelta(t, 0.5*psi*psi/4, cc.crease, 1e-3*0.5*psi*psi/4)
		// σ = 3δ + δ_P + c_J.
		require.GreaterOrEqual(t, cc.sigma(), 3*delta+cc.crease)
		require.InDelta(t, 3*delta+cc.crease, cc.sigma(), 1e-12)
	})

	t.Run("an oblique wall turns the junction by its departure", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, true)
		cc, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.NoError(t, err)
		require.True(t, cc.charged)
		require.Equal(t, 1e-9, cc.delta, "the wall charges are the cylinder's 1e-9 and the extent's 1e-9")
		psi := 4e-6 + (cc.eJ[1]+2e-9)/3
		require.GreaterOrEqual(t, cc.crease, 0.5*psi*psi/4)
	})

	t.Run("a base that states a displacement is charged apart", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, false)
		base.HasAxialDelta, base.AxialDelta = true, 5e-9
		cc, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.NoError(t, err)
		require.Equal(t, 5e-9, cc.deltaP)
		require.GreaterOrEqual(t, cc.sigma(), 3e-9+5e-9+cc.crease)
		require.InDelta(t, 3e-9+5e-9+cc.crease, cc.sigma(), 1e-15)
	})

	t.Run("a base that states a departure is no base", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, false)
		base.TagDeparture, base.NormalBound, base.ExtentDelta = 1e-6, 1e-6, 1e-9
		_, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
	})

	t.Run("a junction whose walls may turn by more than a radian is refused", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, false)
		fp.VertexBound[1] = 5
		_, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "more than a radian")
	})

	t.Run("an open chain on charged walls is refused", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, false)
		cyc.open = true
		_, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "an open tangent chain on walls that state a displacement is not built")
	})

	t.Run("raise widens only the named junction's vertices by its shift", func(t *testing.T) {
		t.Parallel()
		cyc, base, fp, hv, jp := chargeFixture(t, 1e-9, false)
		cc, err := newChainCharges(cyc, base, fp, hv, jp, 0.5)
		require.NoError(t, err)
		w := &work{beta: []float64{1, 1, 1}}
		cc.raise(w, 1, 0, 2)
		require.GreaterOrEqual(t, w.beta[0], 1+cc.eJ[1])
		require.InDelta(t, 1+cc.eJ[1], w.beta[0], 1e-12)
		require.Equal(t, 1.0, w.beta[1])
		require.Equal(t, w.beta[0], w.beta[2])
	})
}

func TestChainAreaCharge(t *testing.T) {
	t.Parallel()
	const (
		tP, tW = 0.5, 0.5
		radius = 7.0
		length = 6.0
		span   = 1.0
		eJ     = 1e-3
		sigma  = 3e-9
	)
	cc := chainCharges{charged: true, delta: 1e-9, eJ: []float64{eJ, eJ}, crease: 0}
	spec := blendSpec{dA: tP, dB: tW}
	entries := []chainEntry{
		{ja: 0, jb: 1, length: length},
		{ja: 1, jb: 0, arc: true, span: span, rim: rim{radius: radius, kappa: -1}},
	}

	t.Run("a chain that reads no charge adds nothing", func(t *testing.T) {
		t.Parallel()
		require.Zero(t, chainCharges{}.areaCharge(entries, tP, tW, spec))
	})

	t.Run("it adds each piece's tube, curvature and junction terms", func(t *testing.T) {
		t.Parallel()
		hyp := math.Hypot(tP, tW)
		rho := radius - tP
		pi := math.Pi
		// The line entry: a wall and a bevel, each a rectangle; the arc entry: a cylinder wall and a cone bevel;
		// the base: the band along both curves.
		lineWall := 2*(length+tW)*2*sigma + (pi+2*pi)*sigma*sigma
		lineBevel := 2*(2*length+2*hyp)*sigma + (pi+2*pi)*sigma*sigma
		arcWall := 2*(2*radius*span+2*tW)*sigma + (pi+2*pi+2*span)*sigma*sigma + radius*span*tW*sigma/radius
		arcBevel := 2*(radius*span+rho*span+2*hyp)*sigma + (pi+2*pi+2*span)*sigma*sigma +
			span*(radius+rho)/2*hyp*sigma/rho
		base := 2*(length+length+radius*span+rho*span)*sigma + (pi+2*(2*pi+span))*sigma*sigma
		tubes := lineWall + lineBevel + arcWall + arcBevel + base
		// The junction terms: width times the larger-radius ratio times the shifts of the junctions it meets.
		factor := radius / rho
		junctions := tW*factor*2*eJ + hyp*factor*2*eJ + tW*factor*2*eJ + hyp*factor*2*eJ + tP*factor*2*eJ

		_, wall, bevel := entries[1].areas(tP, tW, spec)
		_, lineW, lineB := entries[0].areas(tP, tW, spec)
		require.InDelta(t, span*radius*tW, wall, 1e-12)
		require.InDelta(t, length*hyp, lineB, 1e-12)
		require.InDelta(t, length*tW, lineW, 1e-12)
		require.InDelta(t, span*(radius+rho)/2*hyp, bevel, 1e-12)

		cc.delta, cc.crease = sigma/3, 0
		got := cc.areaCharge(entries, tP, tW, spec)
		require.GreaterOrEqual(t, got, tubes+junctions)
		require.InDelta(t, tubes+junctions, got, 1e-6*(tubes+junctions))
		// Without the junction terms it falls short of the sum.
		cc.eJ = []float64{0, 0}
		require.Less(t, cc.areaCharge(entries, tP, tW, spec), tubes*(1+1e-6))
		require.InDelta(t, tubes, cc.areaCharge(entries, tP, tW, spec), 1e-6*tubes)
	})
}

func TestChainJunctionOfObliqueWall(t *testing.T) {
	t.Parallel()
	line, arc, v := chargedJunction(t, 1e-9)

	t.Run("the chain reader proves the junction of a charged wall", func(t *testing.T) {
		t.Parallel()
		_, state, near := tangentStateIn(line, arc, v, tagChain)
		require.Equal(t, clr.DegYes, state)
		require.False(t, near)
		require.NotEqual(t, clr.DegYes, Tangent(line, arc, v), "the strict reader never reads it")
	})

	t.Run("an oblique wall beside a cylinder is a junction in chain mode only", func(t *testing.T) {
		t.Parallel()
		line, arc, v := chargedJunction(t, 1e-9)
		line.Faces[1] = obliquePlane(t, r3.NewVec(1, 0, 0), r3.NewVec(0, 0, 1), 1e-6, 1e-9)
		_, state, _ := tangentStateIn(line, arc, v, tagChain)
		require.Equal(t, clr.DegYes, state)
		for _, mode := range []tagMode{tagExact} {
			_, state, _ := tangentStateIn(line, arc, v, mode)
			require.NotEqual(t, clr.DegYes, state, "mode %d", mode)
		}
	})

	t.Run("a base that states a departure is no chain base", func(t *testing.T) {
		t.Parallel()
		line, arc, v := chargedJunction(t, 0)
		base := line.Faces[0]
		base.TagDeparture, base.NormalBound, base.ExtentDelta = 1e-6, 1e-6, 1e-9
		_, state, _ := tangentStateIn(line, arc, v, tagChain)
		require.Equal(t, clr.DegNo, state)
	})
}
