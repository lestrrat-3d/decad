package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// ratNear asserts an exact rational agrees with a reference float to within a
// few ulps of the reference — the integrals below are exact, so the only gap is
// the reference's own decimal.
func ratNear(t *testing.T, want float64, got *big.Rat) {
	t.Helper()
	f, _ := got.Float64()
	require.InDelta(t, want, f, 1e-15)
}

func TestAbsLinearIntegralMatchesTheClosedForm(t *testing.T) {
	r := func(v float64) *big.Rat { return big.NewRat(0, 1).SetFloat64(v) }

	t.Run("sign-fixed integrand", func(t *testing.T) {
		// f(t) = t + 1 is positive on [0,1]. ∫ f·1 = 3/2, ∫ f·t = 5/6,
		// ∫ f·(1−t) = 2/3.
		ratNear(t, 1.5, absLinearIntegral(r(1), r(1), revolveWeightOne))
		ratNear(t, 5.0/6, absLinearIntegral(r(1), r(1), revolveWeightT))
		ratNear(t, 2.0/3, absLinearIntegral(r(1), r(1), revolveWeightOneMinusT))
	})

	t.Run("sign change inside the interval is decomposed", func(t *testing.T) {
		// f(t) = 2t − 1 crosses zero at t = 1/2. ∫|f| = 1/2, which the SIGNED
		// integral (zero) does not see — the whole point of the decomposition.
		ratNear(t, 0.5, absLinearIntegral(r(2), r(-1), revolveWeightOne))
		// ∫|2t−1|·t dt = 1/24 + 5/24 = 1/4 over [0,1], and the (1−t) weight
		// answers the same by the substitution t → 1 − t. The two weights sum
		// to the unweighted integral, which is the domain split the cell's own
		// fixed diagonal makes.
		ratNear(t, 0.25, absLinearIntegral(r(2), r(-1), revolveWeightT))
		ratNear(t, 0.25, absLinearIntegral(r(2), r(-1), revolveWeightOneMinusT))
	})

	t.Run("root outside the interval keeps one piece", func(t *testing.T) {
		// f(t) = t − 4 is negative throughout; |∫f| = 7/2.
		ratNear(t, 3.5, absLinearIntegral(r(1), r(-4), revolveWeightOne))
	})

	t.Run("against a dense numeric reference", func(t *testing.T) {
		for _, tc := range [][2]float64{{3, -1.25}, {-7, 2}, {0.5, -0.5}, {0, 2}} {
			for _, weight := range []int{revolveWeightOne, revolveWeightT, revolveWeightOneMinusT} {
				const n = 200000
				want := 0.0
				for i := range n {
					x := (float64(i) + 0.5) / n
					w := 1.0
					switch weight {
					case revolveWeightT:
						w = x
					case revolveWeightOneMinusT:
						w = 1 - x
					}
					want += math.Abs(tc[0]*x+tc[1]) * w / n
				}
				got, _ := absLinearIntegral(r(tc[0]), r(tc[1]), weight).Float64()
				require.InDelta(t, want, got, 1e-4, `alpha=%v beta=%v weight=%d`, tc[0], tc[1], weight)
			}
		}
	})
}

func TestRevolveCellAreaSlackBracketsTheTrueJacobianGap(t *testing.T) {
	// A cylinder cell: ρ constant at 4, the meridian chord 3 long, one angular
	// interval of dφ = 2π/16. The true density is L·dφ·ρ throughout, and the
	// held quad is a planar rectangle whose two halves each have twice-area
	// L·chord(4, dφ). The gap is therefore constant and the closed form is
	// exactly |Jtrue − Jheld| over the whole unit square, which the two half
	// domains split evenly.
	const rho, meridianLen, n = 4.0, 3.0, 16
	dPhi := 2 * math.Pi / n
	chord := 2 * rho * math.Sin(dPhi/2)
	jTrue := meridianLen * dPhi * rho
	jHeld := meridianLen * chord
	want := math.Abs(jTrue - jHeld)

	step := intervalScale(twoPiInterval(), big.NewRat(1, n))
	length := pointInterval(big.NewRat(0, 1).SetFloat64(meridianLen))
	twoArea := pointInterval(big.NewRat(0, 1).SetFloat64(jHeld))
	got := revolveCellAreaSlack(rho, rho, length, step, [2]ratInterval{twoArea, twoArea})
	require.InDelta(t, want, got, 1e-9)
	require.Greater(t, got, 0.0)
}

func TestRevolveFanAreaSlackIsHalfTheDensityGap(t *testing.T) {
	// A pole fan: Jtrue = L·dφ·ρ·t and Jheld = 2A·t, so ∫₀¹|c·t| dt = |c|/2 and
	// the pole's own end of the cell makes no difference to the magnitude.
	const rho, meridianLen, n = 5.0, 2.0, 12
	dPhi := 2 * math.Pi / n
	twoArea := 7.0
	c := meridianLen*dPhi*rho - twoArea
	step := intervalScale(twoPiInterval(), big.NewRat(1, n))
	length := pointInterval(big.NewRat(0, 1).SetFloat64(meridianLen))
	area := pointInterval(big.NewRat(0, 1).SetFloat64(twoArea))
	for _, poleFirst := range []bool{true, false} {
		got := revolveFanAreaSlack(rho, poleFirst, length, step, area)
		require.InDelta(t, math.Abs(c)/2, got, 1e-9, `poleFirst=%v`, poleFirst)
	}
}

func TestRevolveAngularSequenceEnclosesItsOwnStoredTrig(t *testing.T) {
	t.Run("a full turn from zero uses exact rational turns", func(t *testing.T) {
		seq, err := revolveAngularSequence(0, 2*math.Pi, true, 12)
		require.NoError(t, err)
		require.Len(t, seq.cos, 12, `a full turn stores no seam sample`)
		require.LessOrEqual(t, seq.gap, revolveTrigGapPrior)
		for l := range seq.cos {
			// The stored pair lies inside the certified enclosure and on the
			// unit circle to within that enclosure's own width.
			require.LessOrEqual(t, intervalFloatError(seq.cosIv[l], seq.cos[l]), revolveTrigGapPrior)
			require.InDelta(t, 1.0, seq.cos[l]*seq.cos[l]+seq.sin[l]*seq.sin[l], 1e-15)
			require.InDelta(t, math.Cos(2*math.Pi*float64(l)/12), seq.cos[l], 1e-12)
			require.InDelta(t, math.Sin(2*math.Pi*float64(l)/12), seq.sin[l], 1e-12)
		}
	})

	t.Run("a partial sweep includes both ends", func(t *testing.T) {
		seq, err := revolveAngularSequence(0.25, 1.5, false, 5)
		require.NoError(t, err)
		require.Len(t, seq.cos, 6)
		require.InDelta(t, math.Cos(0.25), seq.cos[0], 1e-12)
		require.InDelta(t, math.Cos(1.5), seq.cos[5], 1e-12)
		require.InDelta(t, math.Sin(1.5), seq.sin[5], 1e-12)
	})

	t.Run("a non-finite sweep refuses", func(t *testing.T) {
		_, err := revolveAngularSequence(0, math.Inf(1), false, 4)
		require.ErrorIs(t, err, ErrUnsupported)
	})
}

func TestRevolveBudgetReservesBothCoordinateStages(t *testing.T) {
	available, err := revolveBudget(0.1, 1e-9, 2e-9)
	require.NoError(t, err)
	require.Less(t, available, 0.1)
	require.Greater(t, available, 0.09)

	_, err = revolveBudget(1e-9, 1e-9, 0)
	require.ErrorIs(t, err, ErrUnsupported)
	_, err = revolveBudget(1e-9, 2e-9, 0)
	require.ErrorIs(t, err, ErrUnsupported)
}

func TestExactRigidPointRoundIsZeroOnlyForAnExactPlacement(t *testing.T) {
	p := r3.Vec{X: 3.25, Y: -1.5, Z: 7}
	identity := r3.Identity()
	require.Equal(t, 0.0, exactRigidPointRound(identity, p, identity.Apply(p)))

	rot, err := r3.Rotation(r3.Vec{X: 1, Y: 2, Z: 3}, units.Degrees(37))
	require.NoError(t, err)
	require.Positive(t, exactRigidPointRound(rot, p, rot.Apply(p)))

	// A vertex the caller stored WRONG is measured against what the exact map
	// says, so the reading grows with the error rather than hiding it.
	wrong := rot.Apply(p)
	wrong.X += 1e-6
	require.Greater(t, exactRigidPointRound(rot, p, wrong), 1e-6)
}

func TestRevolveIdealPointMeasuresTheConstructionRounding(t *testing.T) {
	// An axis-aligned basis with a power-of-two radius rounds nowhere, so the
	// stored point equals the ideal one exactly.
	b := revolveBasis3Iv{
		a3: mustIvVec(r3.Vec{}),
		w:  mustIvVec(r3.Vec{X: 1}),
		e0: mustIvVec(r3.Vec{Y: 1}),
		e1: mustIvVec(r3.Vec{Z: 1}),
	}
	one := big.NewRat(1, 1)
	zero := new(big.Rat)
	ideal := revolveIdealPoint(b, pointInterval(zero), pointInterval(big.NewRat(8, 1)), pointInterval(one), pointInterval(zero))
	require.Equal(t, 0.0, intervalFloatError(ideal[1], 8.0))

	// A basis whose own construction rounds cannot claim that: the enclosure
	// separates from the stored float and the gap is charged.
	tilted := mustIvVec(r3.Vec{X: 0.1, Y: 0.7, Z: 0.3})
	b2 := revolveBasis3Iv{a3: mustIvVec(r3.Vec{}), w: mustIvVec(r3.Vec{X: 1}), e0: tilted, e1: mustIvVec(r3.Vec{Z: 1})}
	third, ok := new(big.Rat).SetString("1/3")
	require.True(t, ok)
	ideal2 := revolveIdealPoint(b2, pointInterval(zero), pointInterval(third), pointInterval(one), pointInterval(zero))
	held, _ := intervalMid(ideal2[1]).Float64()
	require.Positive(t, intervalFloatError(ideal2[1], math.Nextafter(held, math.Inf(1))))
}

func TestRequireVertexLinksRejectsAPinchedVertex(t *testing.T) {
	// Two tetrahedral cones meeting at one apex: every directed edge is matched,
	// yet the apex's link is two cycles rather than one.
	verts := []r3.Vec{
		{X: 0, Y: 0, Z: 0},
		{X: 1, Y: 0, Z: 1}, {X: -1, Y: 1, Z: 1}, {X: -1, Y: -1, Z: 1},
		{X: 1, Y: 0, Z: -1}, {X: -1, Y: 1, Z: -1}, {X: -1, Y: -1, Z: -1},
	}
	tris := [][3]int{
		{0, 1, 2}, {0, 2, 3}, {0, 3, 1}, {1, 3, 2},
		{0, 5, 4}, {0, 6, 5}, {0, 4, 6}, {4, 5, 6},
	}
	m := &Mesh{vertices: verts, triangles: tris}
	require.NoError(t, requireClosedMesh(m), `the directed-edge audit passes, which is why the link audit exists`)
	err := requireVertexLinks(t.Context(), m)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "cycles rather than one")

	// One cone alone has a sound link at every vertex.
	single := &Mesh{vertices: verts[:4], triangles: tris[:4]}
	require.NoError(t, requireVertexLinks(t.Context(), single))
}

func TestRevolveContactAuditRefusesACrossingPair(t *testing.T) {
	budget := newWorkBudget(t.Context())
	// Two triangles sharing nothing and crossing each other: no separating axis
	// exists, so the audit refuses rather than admitting the pair.
	verts := []r3.Vec{
		{X: -1, Y: 0, Z: 0}, {X: 1, Y: 0, Z: 0}, {X: 0, Y: 2, Z: 0},
		{X: 0, Y: 1, Z: -1}, {X: 0, Y: 1, Z: 1}, {X: 0, Y: -1, Z: 0},
	}
	tris := [][3]int{{0, 1, 2}, {3, 4, 5}}
	err := revolveContactAudit(budget, verts, tris, 0)
	require.ErrorIs(t, err, ErrUnsupported)

	// Move the second triangle clear and the same pair is proven apart.
	apart := append([]r3.Vec(nil), verts...)
	for i := 3; i < 6; i++ {
		apart[i].X += 100
	}
	require.NoError(t, revolveContactAudit(newWorkBudget(t.Context()), apart, tris, 0))
}

func TestRevolveContactAuditRefusesAFacetThinnerThanItsOwnDisplacement(t *testing.T) {
	verts := []r3.Vec{{X: 0}, {X: 1}, {X: 0.5, Y: 1e-12}}
	tris := [][3]int{{0, 1, 2}}
	require.NoError(t, revolveContactAudit(newWorkBudget(t.Context()), verts, tris, 0))
	err := revolveContactAudit(newWorkBudget(t.Context()), verts, tris, 1e-6)
	require.ErrorIs(t, err, ErrUnsupported)
	require.Contains(t, err.Error(), "positive area")
}

func TestRevolveCoordMaxCoversEveryIdealCoordinate(t *testing.T) {
	b := revolveBasis{
		a3: r3.Vec{X: 100, Y: -50, Z: 0},
		w:  r3.Vec{X: 1},
		e0: r3.Vec{Y: 1},
		e1: r3.Vec{Z: 1},
	}
	got := revolveCoordMax(b, 20, 8)
	require.GreaterOrEqual(t, got, 120.0)
	for _, z := range []float64{-20, 0, 20} {
		for i := range 33 {
			phi := float64(i) / 32 * 2 * math.Pi
			p := b.a3.Add(b.w.Scale(z)).Add(b.e0.Scale(8 * math.Cos(phi))).Add(b.e1.Scale(8 * math.Sin(phi)))
			require.LessOrEqual(t, math.Max(math.Abs(p.X), math.Max(math.Abs(p.Y), math.Abs(p.Z))), got)
		}
	}
}

func mustIvVec(v r3.Vec) ivVec3 {
	out, ok := ivVec3Of(v)
	if !ok {
		panic("decad: test vector is not enclosable")
	}
	return out
}

func TestRevolveMeshAreaSlackCoversTheHeldAreaGap(t *testing.T) {
	// docs/tessellation-design.md §14's "check areaSlack against high-precision
	// local area differences": the published slack must cover the gap between
	// the body's own analytic area and the area its facets actually hold.
	for _, tc := range []struct {
		name  string
		build func(*testing.T) (*sketch.Sketch, *sketch.Profile)
	}{
		{"cone with an apex on the axis", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			o := s.CreatePoint(0, 0)
			s.Fix(o)
			apex := s.CreatePoint(10, 0)
			top := s.CreatePoint(0, 5)
			s.CreateLine(o, apex)
			s.CreateLine(apex, top)
			s.CreateLine(top, o)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return s, s.Profiles()[0]
		}},
		{"annular tube clear of the axis", func(t *testing.T) (*sketch.Sketch, *sketch.Profile) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			rect := s.CreateRectangle(0, 5, 10, 15)
			s.Fix(rect.A)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			return s, s.Profiles()[0]
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			s, p := tc.build(t)
			axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
			body, err := New().Revolve(s, p, axis, FullRevolution{})
			require.NoError(t, err)
			mesh, err := body.Tessellate(units.Millimeters(0.1))
			require.NoError(t, err)

			held := 0.0
			for _, tri := range mesh.triangles {
				a, b, c := mesh.vertices[tri[0]], mesh.vertices[tri[1]], mesh.vertices[tri[2]]
				held += b.Sub(a).Cross(c.Sub(a)).Len() / 2
			}
			area, err := body.Area()
			require.NoError(t, err)
			analytic, err := area.Value.In(units.SquareMillimeter)
			require.NoError(t, err)

			gap := math.Abs(analytic - held)
			require.Positive(t, gap, `a chorded revolve holds less area than the surface it stands for`)
			require.Positive(t, mesh.areaSlack)
			require.GreaterOrEqual(t, mesh.areaSlack, gap, `areaSlack must cover the gap it exists to bound`)
			// It is a bound, not an estimate, and §10.2 forbids it from
			// cancelling: a planar annulus cell's true density is LINEAR in the
			// meridian parameter while its flat facet's is constant, so the
			// non-cancelling integral of their difference reads well above the
			// two areas' own near-agreement. It must still stay a small part of
			// the surface it speaks for rather than swamping it.
			require.Less(t, mesh.areaSlack, 0.25*analytic)
		})
	}
}

func TestRevolvePreflightFacetsChargesTheCeilingBeforeAllocating(t *testing.T) {
	// One loop, four meridian samples, three of its four generators sweeping a
	// wall: 6·n wall facets plus the two caps' four. Both §3 ceilings are
	// charged here, before a single vertex is built.
	loop := revLoopMesh{
		resolved: revolveWalks{
			walks: make([]sideWalk, 4),
			kinds: []wallKind{wallCylinder, wallPlane, wallCone, wallAxis},
		},
		samples: []revMeridian{{walk: 0}, {walk: 1}, {walk: 2}, {walk: 3}},
	}
	// The facet-pair ceiling is the binding one: F·(F−1)/2 stays inside
	// maxFacetPairTestsPerCall only up to 4000 facets, which this shape reaches
	// at n = 666 exactly. One angular step more refuses.
	require.NoError(t, revolvePreflightFacets([]revLoopMesh{loop}, 666, false, &revolveWork{}))
	require.ErrorIs(t, revolvePreflightFacets([]revLoopMesh{loop}, 667, false, &revolveWork{}), ErrUnsupported)
	require.ErrorIs(t, revolvePreflightFacets([]revLoopMesh{loop}, 10923, false, &revolveWork{}), ErrUnsupported)

	// A pole ring fans rather than quads, so its own cell costs half as many
	// facets per angular step and the same walks admit a finer count.
	poled := loop
	poled.samples = []revMeridian{{walk: 0, onAxis: true}, {walk: 1}, {walk: 2}, {walk: 3}}
	require.NoError(t, revolvePreflightFacets([]revLoopMesh{poled}, 799, false, &revolveWork{}))
	require.ErrorIs(t, revolvePreflightFacets([]revLoopMesh{poled}, 800, false, &revolveWork{}), ErrUnsupported)

	// A refinement retry never resets the cumulative counters
	// (docs/tessellation-design.md §3): the same shape that fits on its own is
	// refused once an earlier attempt has already spent most of the ceiling.
	work := &revolveWork{}
	require.NoError(t, revolvePreflightFacets([]revLoopMesh{loop}, 600, false, work))
	require.Positive(t, work.facets)
	require.Positive(t, work.pairs)
	require.ErrorIs(t, revolvePreflightFacets([]revLoopMesh{loop}, 600, false, work), ErrUnsupported)
}

func TestCheckedIntegerArithmeticRefusesOverflow(t *testing.T) {
	sum, ok := addChecked(math.MaxUint64, 1)
	require.False(t, ok)
	require.Equal(t, uint64(0), sum)
	sum, ok = addChecked(7, 5)
	require.True(t, ok)
	require.Equal(t, uint64(12), sum)

	_, ok = mulChecked(math.MaxUint64/2+1, 3)
	require.False(t, ok)
	product, ok := mulChecked(6, 7)
	require.True(t, ok)
	require.Equal(t, uint64(42), product)
	product, ok = mulChecked(0, math.MaxUint64)
	require.True(t, ok)
	require.Equal(t, uint64(0), product)
}

func TestRevolveMeshCarriesNoOccupiedVolumeProof(t *testing.T) {
	// docs/tessellation-design.md §11: a revolve mesh serves export and stays
	// out of the boolean until T4. Both gates say so — the payload-class
	// pre-check that spares the mesh, and operandSymDiff on the mesh itself.
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 10, 8)
	s.Fix(rect.A)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	axis := SketchLine{Start: Point2{U: 0, V: 0}, End: Point2{U: 1, V: 0}}
	body, err := New().Revolve(s, s.Profiles()[0], axis, FullRevolution{})
	require.NoError(t, err)

	mesh, err := body.Tessellate(units.Millimeters(0.2))
	require.NoError(t, err)
	require.False(t, mesh.symDiffOK)
	_, err = operandSymDiff(mesh)
	require.ErrorIs(t, err, ErrUnsupported)
	require.ErrorIs(t, requireVolumeProvingPayload(body, 0), ErrUnsupported)

	// Every source face still publishes a positive displacement, so the mesh is
	// a complete export operand even though no boolean may compose it.
	require.Positive(t, mesh.bound)
	for _, f := range mesh.source {
		d, ok := mesh.sourceBound(f)
		require.True(t, ok)
		require.Positive(t, d)
		require.LessOrEqual(t, d, mesh.bound)
	}
}

// TestRevolveVertexIsolatedDecidesACapFanAgainstTheNextChordWall pins the pair
// the edge-pair separating plane exists for, at the exact coordinates a
// partial-sweep groove produced: facet 60, a wall triangle of one meridian
// chord, against facet 167, a start-cap fan triangle, sharing vertex 42 alone.
//
// None of the four candidates built from ONE triangle's own normal decides it.
// With the wall as a, the cap's two corners read mixed signs on all four. With
// the cap as a, its own normal reads the wall's in-plane corner at −7.1e-15 —
// that corner lies in the cap plane without being one of the cap triangle's own
// vertices, so the reading is a numerical zero the perturbation allowance
// cannot sign — and every rotation of that normal reads the wall's two corners
// with opposite signs, because the off-plane corner's in-plane component is
// −ρ(1−cos dφ)·e(φ0), whose sign is fixed. Refining the angular sequence
// shrinks that component as dφ² and never flips it, so the whole refinement
// chain fails identically and the mesh refuses however fine the chording.
//
// The edge-pair family decides it: g = eA × eB reads the two remaining corners
// at +38.4 and −31.7, four hundred million times its own perturbation
// allowance.
func TestRevolveVertexIsolatedDecidesACapFanAgainstTheNextChordWall(t *testing.T) {
	verts := make([]r3.Vec, 50)
	verts[35] = r3.Vec{X: -22.861137887942604, Y: 3.1848000973097195, Z: -11.332206870439578}
	verts[42] = r3.Vec{X: -21.72779063345132, Y: 2.943952572193523, Z: -10.587186732066304}
	verts[43] = r3.Vec{X: -22.534312279222867, Y: 0.8079815808422071, Z: -9.77332215919684}
	verts[7] = r3.Vec{X: -29.408326745936623, Y: 7.794809887107666, Z: -2.8190966120010716}
	verts[49] = r3.Vec{X: -20.44213746589201, Y: 2.524200990919139, Z: -10.325570450153995}

	const delta = 7.150045910436396e-15
	triA := [3]int{35, 42, 43} // the wall triangle of one meridian chord
	triB := [3]int{7, 49, 42}  // the start cap's fan triangle
	a, ok := newRevolveAuditTri(verts, triA)
	require.True(t, ok)
	b, ok := newRevolveAuditTri(verts, triB)
	require.True(t, ok)

	require.True(t,
		revolveVertexIsolated(a, triA, b, triB, 42, delta) ||
			revolveVertexIsolated(b, triB, a, triA, 42, delta),
		`the cap fan and the next chord's wall must be proven to meet only at the vertex they share`)

	// The same pair inside the whole audit, which is where the refusal used to
	// surface. Facet indices are the audit's own, so the two triangles are
	// handed to it alone.
	require.NoError(t, revolveContactAudit(newWorkBudget(t.Context()), verts, [][3]int{triA, triB}, delta))
}
