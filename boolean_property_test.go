package decad_test

import (
	"errors"
	"math"
	"math/rand"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file hardens the mesh boolean (docs/evaluator-design.md §9) with
// randomized, seeded property tests and a native fuzz target. The rest of the
// suite is fixed hand-chosen cases; the reviewers flagged the degenerate-case
// space as hard to exhaust by inspection, so here we sample it at scale and
// assert the invariants that must hold whichever degenerate case is hit:
//
//  1. watertightness/manifoldness of every successful result (the edge-pairing
//     audit, reusing requireWatertight);
//  2. bound soundness — the reported volume/area/centroid bound provably covers
//     the deviation from an INDEPENDENTLY computed true value (inclusion-
//     exclusion over axis-aligned boxes, exact in closed form). This is the
//     cardinal property: a bound that failed to cover the truth would be a
//     false certification;
//  3. algebraic identities (union commutative, disjoint union = exact sum,
//     A∩B ⊆ A);
//  4. refusal soundness — a true tangency/coincident-face pair is refused with
//     a known sentinel, never silently accepted, and a clean transversal pair
//     is never spuriously refused.
//
// All randomness is drawn from a fixed seed and the seed is logged, so any CI
// failure is replayable. Coordinates are kept small so the inclusion-exclusion
// oracle is exact in float64 (products and sums stay well inside 2^53).

// boolPropSeed seeds every randomized test in this file. Fixed for
// reproducibility; logged by each test so a failure can be replayed.
const boolPropSeed = 0x0DECAD

// abox is an axis-aligned box in world space, lo < hi on every axis.
type abox struct{ lo, hi r3.Vec }

func (a abox) volume() float64 {
	return (a.hi.X - a.lo.X) * (a.hi.Y - a.lo.Y) * (a.hi.Z - a.lo.Z)
}

func (a abox) center() r3.Vec {
	return r3.NewVec((a.lo.X+a.hi.X)/2, (a.lo.Y+a.hi.Y)/2, (a.lo.Z+a.hi.Z)/2)
}

// abConfig bundles a boolean op with its two operand boxes for failure
// messages: a require failure prints the exact configuration that hit it.
type abConfig struct {
	op   decad.OpKind
	a, b abox
}

// overlap is the axis-aligned intersection box of a and b, plus its volume.
func overlap(a, b abox) (abox, float64) {
	lo := r3.NewVec(math.Max(a.lo.X, b.lo.X), math.Max(a.lo.Y, b.lo.Y), math.Max(a.lo.Z, b.lo.Z))
	hi := r3.NewVec(math.Min(a.hi.X, b.hi.X), math.Min(a.hi.Y, b.hi.Y), math.Min(a.hi.Z, b.hi.Z))
	v := math.Max(0, hi.X-lo.X) * math.Max(0, hi.Y-lo.Y) * math.Max(0, hi.Z-lo.Z)
	return abox{lo, hi}, v
}

// trueBoolean returns the exact measure-theoretic volume and centroid of the
// boolean of two axis-aligned boxes. Exact in closed form by inclusion-
// exclusion over axis-aligned boxes; the shared boundary is a zero-measure set,
// so a touching contact contributes nothing.
func trueBoolean(op decad.OpKind, a, b abox) (float64, r3.Vec) {
	o, ov := overlap(a, b)
	va, vb := a.volume(), b.volume()
	var vol float64
	var cen r3.Vec
	switch op {
	case decad.OpUnion:
		vol = va + vb - ov
		if vol > 1e-12 {
			m := a.center().Scale(va).Add(b.center().Scale(vb)).Sub(o.center().Scale(ov))
			cen = m.Scale(1 / vol)
		}
	case decad.OpIntersect:
		vol = ov
		cen = o.center()
	case decad.OpCut:
		vol = va - ov
		if vol > 1e-12 {
			m := a.center().Scale(va).Sub(o.center().Scale(ov))
			cen = m.Scale(1 / vol)
		}
	}
	return vol, cen
}

// makeBox builds an axis-aligned box body from a in doc: a rectangle extruded
// along +z from z = 0, translated to lo.Z when that is nonzero.
func makeBox(t *testing.T, doc *decad.Document, a abox) *decad.Body {
	t.Helper()
	body := boxBody(t, doc, a.lo.X, a.lo.Y, a.hi.X, a.hi.Y, a.hi.Z-a.lo.Z)
	if a.lo.Z != 0 {
		body = translated(t, body, 0, 0, a.lo.Z)
	}
	return body
}

// runBool dispatches the op over two live bodies.
func runBool(op decad.OpKind, a, b *decad.Body) (*decad.Body, error) {
	switch op {
	case decad.OpUnion:
		return decad.Union(a, b)
	case decad.OpIntersect:
		return decad.Intersect(a, b)
	default:
		return decad.Cut(a, b)
	}
}

// boolSentinels are the only errors a boolean may return for a real geometric
// configuration: a contact the exact predicates cannot classify (ErrDegenerate),
// a proximity/staging refusal (ErrUnsupported), or an empty result
// (ErrBooleanFailed). Any other error — or a panic — is a genuine finding.
var boolSentinels = []error{decad.ErrDegenerate, decad.ErrUnsupported, decad.ErrBooleanFailed}

func isBoolSentinel(err error) bool {
	for _, s := range boolSentinels {
		if errors.Is(err, s) {
			return true
		}
	}
	return false
}

// requireVolumeSound asserts the reported volume is a non-negative magnitude
// and that its bound covers the deviation from the independently known true
// volume. slack absorbs the oracle's own float rounding (a rigid-motion oracle
// rounds where the product code does not); it is set to zero for the exact
// axis-aligned path so a wrong exact volume cannot slip through.
func requireVolumeSound(t *testing.T, got *decad.Body, trueVol, slack float64, cfg abConfig) {
	t.Helper()
	vol, err := got.Volume()
	require.NoError(t, err, `%+v`, cfg)
	v := volumeMM(t, vol)
	require.GreaterOrEqual(t, v, -1e-9, `a reported volume is a magnitude: %+v`, cfg)
	require.LessOrEqual(t, math.Abs(v-trueVol), boundMM3(t, vol)+slack,
		`the volume bound must cover the true volume (got %v, true %v, bound %v): %+v`,
		v, trueVol, boundMM3(t, vol), cfg)
	if vol.Exactness == decad.Exact {
		require.Zero(t, boundMM3(t, vol), `an Exact volume carries a zero bound: %+v`, cfg)
	}
}

// requireCentroidSound asserts the reported centroid lies within its own bound
// of the independently known true centroid.
func requireCentroidSound(t *testing.T, got *decad.Body, trueCen r3.Vec, cfg abConfig) {
	t.Helper()
	cen, err := got.Centroid()
	require.NoError(t, err, `%+v`, cfg)
	// A tiny relative slack: the oracle divides moment by volume in float64,
	// so its own centroid carries ~1e-13 mm of rounding the product bound does
	// not answer for. Gross false certifications are orders larger.
	slack := 1e-9 * (1 + trueCen.Len())
	dist := cen.Value.Sub(trueCen).Len()
	require.LessOrEqual(t, dist, cen.Bound.Base()+slack,
		`the centroid bound must cover the true centroid (dist %v, bound %v): %+v`,
		dist, cen.Bound.Base(), cfg)
}

// cleanTransversalPair draws two axis-aligned boxes that overlap positively on
// every axis with all four coordinates distinct per axis — general position, so
// no face plane coincides and the boolean is guaranteed to succeed. Every
// coordinate is a small integer, so the inclusion-exclusion oracle is exact in
// float64 and every all-planar boolean keeps an Exact volume.
func cleanTransversalPair(rng *rand.Rand) (abox, abox) {
	axis := func() (float64, float64, float64, float64) {
		a0 := float64(rng.Intn(8))
		aLen := float64(4 + rng.Intn(9)) // 4..12
		a1 := a0 + aLen
		b0 := a0 + float64(1+rng.Intn(int(aLen)-1)) // strictly inside (a0, a1)
		b1 := b0 + float64(4+rng.Intn(9))
		if b1 == a1 {
			b1++
		}
		return a0, a1, b0, b1
	}
	ax0, ax1, bx0, bx1 := axis()
	ay0, ay1, by0, by1 := axis()
	az0, az1, bz0, bz1 := axis()
	return abox{r3.NewVec(ax0, ay0, az0), r3.NewVec(ax1, ay1, az1)},
		abox{r3.NewVec(bx0, by0, bz0), r3.NewVec(bx1, by1, bz1)}
}

func TestBooleanBoundSoundnessAxisAligned(t *testing.T) {
	t.Logf("boolPropSeed=%#x", boolPropSeed)
	rng := rand.New(rand.NewSource(boolPropSeed))
	ops := []decad.OpKind{decad.OpUnion, decad.OpIntersect, decad.OpCut}

	// Axis-aligned geometry is the hard case for the boolean's axis-ray parity
	// classifier: an axis ray from a seed point can graze the coplanar faces of
	// an axis-aligned mesh, and when ALL SIX axis directions are ambiguous the
	// boolean refuses LOUDLY (ErrBooleanFailed) rather than guess — a sound,
	// documented refusal, never a wrong answer (boolean_exact.go). So a refusal
	// here is acceptable and the document must stay untouched; the strict
	// "a clean transversal pair is never spuriously refused" guarantee lives in
	// the rotated loop below, where the classifier's rays are non-degenerate.
	for iter := range 60 {
		a, b := cleanTransversalPair(rng)
		for _, op := range ops {
			cfg := abConfig{op, a, b}
			doc := decad.New()
			ba := makeBox(t, doc, a)
			bb := makeBox(t, doc, b)

			got, err := runBool(op, ba, bb)
			if err != nil {
				require.Truef(t, isBoolSentinel(err),
					`iter %d: an axis-aligned overlap may only refuse with a known sentinel, got %v: %+v`, iter, err, cfg)
				require.Len(t, doc.Bodies(), 2, `a refused boolean leaves the document untouched: %+v`, cfg)
				continue
			}

			requireBodyWatertight(t, got)

			trueVol, trueCen := trueBoolean(op, a, b)
			requireVolumeSound(t, got, trueVol, 0, cfg)
			requireCentroidSound(t, got, trueCen, cfg)

			// The intersection of two axis-aligned boxes is itself an axis-
			// aligned box, so its true surface area is closed form too — an
			// independent area oracle.
			if op == decad.OpIntersect {
				o, _ := overlap(a, b)
				dx, dy, dz := o.hi.X-o.lo.X, o.hi.Y-o.lo.Y, o.hi.Z-o.lo.Z
				trueArea := 2 * (dx*dy + dy*dz + dz*dx)
				area, err := got.Area()
				require.NoError(t, err, `%+v`, cfg)
				require.LessOrEqual(t, math.Abs(areaMM2(t, area)-trueArea),
					area.Bound.Base()+1e-6*(1+trueArea),
					`the area bound must cover the true surface area: %+v`, cfg)
			}
		}
	}
}

func TestBooleanBoundSoundnessRotated(t *testing.T) {
	t.Logf("boolPropSeed=%#x", boolPropSeed)
	rng := rand.New(rand.NewSource(boolPropSeed + 1))
	ops := []decad.OpKind{decad.OpUnion, decad.OpIntersect, decad.OpCut}

	for iter := range 40 {
		a, b := cleanTransversalPair(rng)

		// One rigid motion applied to BOTH operands. A rigid motion preserves
		// volume and centroid up to its own image, so the axis-aligned oracle
		// still holds — but the rotated contacts no longer round exactly, so
		// the reported volume is Approximate and its bound is the thing under
		// test. A rigid image of a general-position pair is still general
		// position, so the boolean must not be spuriously refused either.
		angle := (rng.Float64()*2 - 1) * math.Pi
		axisV := r3.NewVec(rng.Float64()*2-1, rng.Float64()*2-1, rng.Float64()*2-1)
		if axisV.Len() < 1e-6 {
			axisV = r3.NewVec(0, 0, 1)
		}
		rot, err := r3.Rotation(axisV, units.Radians(angle))
		require.NoError(t, err)
		trans, err := r3.Translation(r3.NewVec((rng.Float64()*2-1)*10, (rng.Float64()*2-1)*10, (rng.Float64()*2-1)*10))
		require.NoError(t, err)
		xform, err := rot.Then(trans)
		require.NoError(t, err)

		for _, op := range ops {
			cfg := abConfig{op, a, b}
			doc := decad.New()
			ba, err := makeBox(t, doc, a).Placed(xform)
			require.NoError(t, err)
			bb, err := makeBox(t, doc, b).Placed(xform)
			require.NoError(t, err)

			got, err := runBool(op, ba, bb)
			require.NoErrorf(t, err, `iter %d: a rotated general-position overlap must not be refused: %+v`, iter, cfg)

			requireBodyWatertight(t, got)

			trueVol, trueCenLocal := trueBoolean(op, a, b)
			trueCen := xform.Apply(trueCenLocal)
			// Slack for the oracle: my trueVol is the perfect-rotation volume,
			// while Placed rounds the rotation, so the actual solid's volume
			// differs by ~1e-13·volume that the reported bound does not (and
			// need not) answer for. A false certification is orders larger.
			requireVolumeSound(t, got, trueVol, 1e-7*(1+math.Abs(trueVol)), cfg)
			requireCentroidSound(t, got, trueCen, cfg)
		}
	}
}

func TestBooleanAlgebraicIdentities(t *testing.T) {
	t.Logf("boolPropSeed=%#x", boolPropSeed)
	rng := rand.New(rand.NewSource(boolPropSeed + 2))

	// These identities need a SUCCESSFUL boolean to compare. Axis-aligned pairs
	// occasionally hit the sound all-rays-ambiguous refusal (see the bound-
	// soundness test); those iterations carry no comparison and are skipped —
	// the refusal itself is exercised elsewhere. checked counts the comparisons
	// actually made, so a run that silently skipped everything still fails.
	t.Run("union commutative", func(t *testing.T) {
		checked := 0
		for range 40 {
			a, b := cleanTransversalPair(rng)
			d1 := decad.New()
			ab, err := decad.Union(makeBox(t, d1, a), makeBox(t, d1, b))
			if err != nil {
				require.True(t, isBoolSentinel(err))
				continue
			}
			d2 := decad.New()
			ba, err := decad.Union(makeBox(t, d2, b), makeBox(t, d2, a))
			if err != nil {
				require.True(t, isBoolSentinel(err))
				continue
			}
			checked++

			vab, err := ab.Volume()
			require.NoError(t, err)
			vba, err := ba.Volume()
			require.NoError(t, err)
			// Integer axis-aligned unions are Exact, so the two orders agree
			// exactly; the bound sum is a safety net, not a crutch.
			require.LessOrEqual(t, math.Abs(volumeMM(t, vab)-volumeMM(t, vba)),
				boundMM3(t, vab)+boundMM3(t, vba),
				`A∪B and B∪A enclose the same volume`)
		}
		require.Positive(t, checked, `at least one commutative pair must have been compared`)
	})

	t.Run("disjoint union is the exact sum", func(t *testing.T) {
		for range 30 {
			// Two boxes with a positive gap along x: disjoint, so the union is
			// two lumps whose volume is the exact sum, nothing chorded.
			a := abox{r3.NewVec(0, 0, 0), r3.NewVec(float64(3+rng.Intn(6)), float64(3+rng.Intn(6)), float64(3+rng.Intn(6)))}
			gap := float64(1 + rng.Intn(5))
			bx0 := a.hi.X + gap
			b := abox{r3.NewVec(bx0, 0, 0), r3.NewVec(bx0+float64(3+rng.Intn(6)), float64(3+rng.Intn(6)), float64(3+rng.Intn(6)))}

			doc := decad.New()
			got, err := decad.Union(makeBox(t, doc, a), makeBox(t, doc, b))
			require.NoError(t, err)
			require.Len(t, got.Lumps(), 2, `disjoint operands union to two lumps`)
			vol, err := got.Volume()
			require.NoError(t, err)
			require.Equal(t, decad.Exact, vol.Exactness, `a disjoint union chords nothing`)
			require.InDelta(t, a.volume()+b.volume(), volumeMM(t, vol), 1e-9,
				`disjoint union volume is the exact sum`)
		}
	})

	t.Run("intersection is a subset", func(t *testing.T) {
		checked := 0
		for range 40 {
			a, b := cleanTransversalPair(rng)
			doc := decad.New()
			got, err := decad.Intersect(makeBox(t, doc, a), makeBox(t, doc, b))
			if err != nil {
				require.True(t, isBoolSentinel(err))
				continue
			}
			checked++
			vol, err := got.Volume()
			require.NoError(t, err)
			minOperand := math.Min(a.volume(), b.volume())
			require.LessOrEqual(t, volumeMM(t, vol), minOperand+boundMM3(t, vol),
				`A∩B cannot exceed the smaller operand`)
		}
		require.Positive(t, checked, `at least one intersection must have been compared`)
	})
}

// degenerateCoord draws a coordinate from a tiny discrete set, so pairs
// frequently share faces, edges and corners, nest, or touch — the degenerate
// cluster the reviewers named.
func degenerateBox(rng *rand.Rand) abox {
	set := []float64{0, 3, 6, 9}
	pick := func() (float64, float64) {
		for {
			p, q := set[rng.Intn(len(set))], set[rng.Intn(len(set))]
			if p != q {
				return math.Min(p, q), math.Max(p, q)
			}
		}
	}
	x0, x1 := pick()
	y0, y1 := pick()
	z0, z1 := pick()
	return abox{r3.NewVec(x0, y0, z0), r3.NewVec(x1, y1, z1)}
}

func TestBooleanDegenerateClusterSoundness(t *testing.T) {
	t.Logf("boolPropSeed=%#x", boolPropSeed)
	rng := rand.New(rand.NewSource(boolPropSeed + 3))
	ops := []decad.OpKind{decad.OpUnion, decad.OpIntersect, decad.OpCut}

	// Every coordinate is drawn from {0,3,6,9}, so coincident faces, shared
	// edges, corner/edge kisses, nesting and disjoint-touching all arise. For
	// each configuration the boolean must EITHER succeed soundly (watertight,
	// its bound covering the measure-theoretic true volume) OR refuse with a
	// known sentinel. It must never return a wrong volume or an unexpected
	// error, whichever degenerate case it landed on.
	for iter := range 300 {
		a := degenerateBox(rng)
		b := degenerateBox(rng)
		for _, op := range ops {
			cfg := abConfig{op, a, b}
			doc := decad.New()
			ba := makeBox(t, doc, a)
			bb := makeBox(t, doc, b)

			got, err := runBool(op, ba, bb)
			if err != nil {
				require.Truef(t, isBoolSentinel(err),
					`iter %d: a degenerate boolean must refuse with a known sentinel, got %v: %+v`, iter, err, cfg)
				// A refused boolean leaves both operands live and no step recorded.
				require.Len(t, doc.Bodies(), 2, `a refused boolean leaves the document untouched: %+v`, cfg)
				continue
			}

			requireBodyWatertight(t, got)
			trueVol, _ := trueBoolean(op, a, b)
			// A touching contact is a measure-zero boundary, so if the boolean
			// accepts it the reported volume still matches inclusion-exclusion.
			requireVolumeSound(t, got, trueVol, 1e-6*(1+math.Abs(trueVol)), cfg)
		}
	}
}

func TestBooleanRefusalSoundnessTangentCylinder(t *testing.T) {
	t.Logf("boolPropSeed=%#x", boolPropSeed)
	rng := rand.New(rand.NewSource(boolPropSeed + 4))

	// A Ø(2r) cylinder placed relative to the plate's x = 20 face. Its true
	// surface is tangent to that face when the centre sits at x = 20 + r; the
	// chord polygon lies strictly inside the true cylinder, so the tangency
	// can vanish from the tessellation and the verdict would be decided by
	// where the chords fell. The proximity gate refuses that question
	// (ErrUnsupported). Moved clearly clear, the union is two lumps; moved
	// clearly overlapping (a rod piercing the plate), it is one lump. Refusal
	// soundness in both directions: the exact tangency refuses, the decidable
	// placements do not.
	//
	// The cylinder is dropped to z −6..14 so it clears the plate's z = 0 and
	// z = 8 cap planes: a shared cap plane would be a real face-on-face contact
	// and would mask the x-direction behaviour under test.
	makeCyl := func(doc *decad.Document, cx, cy, r float64) *decad.Body {
		return translated(t, diskBody(t, doc, cx, cy, r), 0, 0, -6)
	}
	// Kept to a few iterations: the evaluator-internal chord tolerance
	// tessellates the cylinder finely (hundreds of facets), so each curved
	// boolean is comparatively heavy. A handful of radii spans the behaviour.
	for range 4 {
		r := 4.0 + rng.Float64()*4 // 4..8

		// Exact tangency along the x = 20 wall: refused. Whether the pre-
		// tessellation proximity gate catches it (ErrUnsupported) or the mesh
		// pass sees an edge grazing the plate facet (ErrDegenerate) depends on
		// where a chord vertex fell — but a tangency is refused EITHER way,
		// never silently accepted as a clean two-lump body.
		doc := decad.New()
		plate := boxBody(t, doc, 0, 0, 20, 20, 8)
		cyl := makeCyl(doc, 20+r, 10, r)
		_, err := decad.Union(plate, cyl)
		require.Truef(t, errors.Is(err, decad.ErrUnsupported) || errors.Is(err, decad.ErrDegenerate),
			`an exact tangency must be refused, got %v (r=%v)`, err, r)
		require.Len(t, doc.Bodies(), 2)

		// Clearly clear: a decidable disjoint union of two lumps.
		gap := 1.0 + rng.Float64()*3
		doc2 := decad.New()
		plate2 := boxBody(t, doc2, 0, 0, 20, 20, 8)
		apart := makeCyl(doc2, 20+r+gap, 10, r)
		got, err := decad.Union(plate2, apart)
		require.NoErrorf(t, err, `a clearly separated pair is decidable (r=%v gap=%v)`, r, gap)
		require.Len(t, got.Lumps(), 2)
		requireBodyWatertight(t, got)

		// Clearly overlapping: a rod piercing the plate, one lump, whose deep
		// crossing the chords cannot hide.
		over := 1.5 + rng.Float64()*2
		doc3 := decad.New()
		plate3 := boxBody(t, doc3, 0, 0, 20, 20, 8)
		into := makeCyl(doc3, 20-over, 10, r)
		got3, err := decad.Union(plate3, into)
		require.NoErrorf(t, err, `a clearly overlapping pair is decidable (r=%v over=%v)`, r, over)
		require.Len(t, got3.Lumps(), 1)
		requireBodyWatertight(t, got3)
	}
}

// FuzzBoolean drives the boolean over two axis-aligned boxes whose twelve
// coordinates are decoded from the fuzz input. Every reachable configuration
// must satisfy the same contract as the seeded tests: a success is watertight
// with a bound covering the true volume, a failure is a known sentinel. The
// corpus seeds cover an overlap, a disjoint pair and a nesting.
func FuzzBoolean(f *testing.F) {
	f.Add([]byte{0, 0, 0, 4, 4, 4, 2, 2, 2, 6, 6, 6, 0})  // overlapping
	f.Add([]byte{0, 0, 0, 3, 3, 3, 8, 0, 0, 12, 3, 3, 1}) // disjoint along x
	f.Add([]byte{0, 0, 0, 9, 9, 9, 3, 3, 3, 6, 6, 6, 2})  // b nested in a
	f.Add([]byte{0, 0, 0, 4, 4, 4, 4, 0, 0, 8, 4, 4, 0})  // shared face plane

	f.Fuzz(func(t *testing.T, data []byte) {
		if len(data) < 13 {
			return
		}
		// Decode two boxes from small even coordinates; skip any collapsed axis.
		box := func(off int) (abox, bool) {
			c := [6]float64{}
			for i := range 6 {
				c[i] = float64(data[off+i] % 7) // 0..6
			}
			lo := r3.NewVec(math.Min(c[0], c[3]), math.Min(c[1], c[4]), math.Min(c[2], c[5]))
			hi := r3.NewVec(math.Max(c[0], c[3]), math.Max(c[1], c[4]), math.Max(c[2], c[5]))
			if hi.X-lo.X < 1 || hi.Y-lo.Y < 1 || hi.Z-lo.Z < 1 {
				return abox{}, false
			}
			return abox{lo, hi}, true
		}
		a, okA := box(0)
		b, okB := box(6)
		if !okA || !okB {
			return
		}
		ops := []decad.OpKind{decad.OpUnion, decad.OpIntersect, decad.OpCut}
		op := ops[int(data[12])%len(ops)]
		cfg := abConfig{op, a, b}

		doc := decad.New()
		got, err := runBool(op, makeBox(t, doc, a), makeBox(t, doc, b))
		if err != nil {
			require.Truef(t, isBoolSentinel(err), `unexpected error %v: %+v`, err, cfg)
			return
		}
		requireBodyWatertight(t, got)
		trueVol, _ := trueBoolean(op, a, b)
		requireVolumeSound(t, got, trueVol, 1e-6*(1+math.Abs(trueVol)), cfg)
	})
}
