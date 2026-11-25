package meshbool

import (
	"context"
	"errors"
	"math"
	"math/big"
	"runtime"
	"slices"
	"sync"
	"sync/atomic"

	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file lets the rigid copies of one faceted mesh inherit a proof of
// embedding from their source instead of each being scanned again
// (agent-docs/evaluator-design.md §9, "A rigid copy inherits its source's
// separation"). Everything below is reject-only: a certificate that does not
// apply, or a check that does not pass, leaves the caller to run the held
// embedding scan on the copy unchanged, so a certified copy returns exactly
// what that scan would have returned (no moves, no repair summary).
//
// Setting. A copy's vertex W_i is the nearest float to the exact image
// X_i = D v_i + t of source vertex v_i. The copy's index structure is the
// source's (a reflection swaps two corners of each facet).
//
// Lemma 1 (pull back). Let e be the entrywise absolute sum of DᵀD − I, which
// bounds the spectral norm of DᵀD − I, so every eigenvalue of DᵀD lies in
// [1−e, 1+e]. For e < 1/2, D is invertible and |D⁻¹x| ≤ |x|/√(1−e) ≤ (1+e)|x|
// (the second step holds for e ≤ 0.618). Put S'_i = D⁻¹(W_i − t). Then
// W_i = D S'_i + t exactly, and |S'_i − v_i| = |D⁻¹(W_i − X_i)| ≤ (1+e)·δ_i,
// where δ_i bounds |W_i − X_i|. Placement rounds the exact image once, so each
// coordinate errs by at most half an ulp of X_i's coordinate. A row of D has
// 1-norm at most 3√(1+e) < 3.7, so |X_i|∞ ≤ 3.7·m + |t|∞ ≤ 2(2m + |t|∞) for
// m = max|v_i|∞, hence half an ulp of it is at most ulp(2m + |t|∞), and the
// 3D error is at most √3 times that, which RigidRoundAllow (16 ulps of
// 2m + |t|∞, times √3, rounded up) dominates. The caller passes the largest
// δ_i it charged; ε' = (1+e)·max δ_i, rounded up, bounds every |S'_i − v_i| and
// hence every coordinate of that difference.
//
// The pull back is a bijective affine map A(x) = D x + t, so for any two
// triangles A(T) ∩ A(U) = A(T ∩ U): the copy's facets meet exactly as the facets
// on S' do, with the same indices. The certificate decides every facet pair of
// the source into exactly one of three sets.
//
//	adjacent  the pair shares at least one vertex index (one, two or three, a
//	          duplicate facet included). Every adjacent pair is checked exactly
//	          on each copy with pairEmbedded, today's predicate.
//	certified the pair shares no index and a separating direction n of the
//	          source gap G exceeds 2τ|n|, by the test below, or the pair's
//	          boxes padded by 2τ do not overlap.
//	thin      every other pair. Each copy checks it with pairEmbedded.
//
// Lemma 2 (gap shrink). A point of a triangle is a convex combination of its
// corners, so moving each corner by at most ε' ≤ τ in 3D moves each point by at
// most ε'. For a float vector n, if max over T of n·x + G ≤ min over U of n·y,
// then the moved triangles keep a gap of at least G − 2ε'|n|₂ along n, and are
// disjoint when G > 2ε'|n|₂. Two triangles whose boxes padded by 2τ (rounded
// outward) do not overlap are more than 4τ apart along one axis, hence more
// than 2τ ≥ 2ε' apart after the move. So a certified pair is disjoint on S', so
// on the copy, and so is embedded (no shared index means no contact is allowed).
//
// Float error. The test reads each corner's offset from one corner of the pair,
// so the error scales with the pair, not with the coordinates. A dot product
// n·d of three products and two sums, with d a float difference, differs from
// the exact n·(x−o) by at most 6u·Σ|n_k||d_k| with u = 2⁻⁵³ (the differences
// contribute u|d_k|); the test charges 1e-12·Σ|n_k||d_k|, about 2000 times
// that, to each of the six dot products, and charges 2τ(1+1e-9)·|n|₁(1+1e-12)
// for the perturbation (|n|₂ ≤ |n|₁). Every comparison is written so that a
// NaN fails it, and every difference and every n component must be zero or lie
// in [2⁻³⁰⁰, 2²⁵⁰] and [2⁻⁴⁰⁰, 2⁴⁰⁰] so that no product underflows or
// overflows; anything else is not certified.
//
// What a certified copy skips. Today's scan checks the pairs that involve a
// facet with a moved vertex and returns zero moves when each is embedded. A
// copy whose adjacent and thin pairs and facet openness all pass here has every
// pair embedded, so today's scan finds no failing pair provided its predicates
// are complete on embedded pairs: pairEmbeddedPath returns false only for three
// shared vertices, a zero normal, or a contact the exact classifier finds, and
// TriTriClassifyCore reports an error only for a facet crossing a plane more
// than twice, which an embedded pair cannot do. A copy this check refuses runs
// the scan.
//
// Refusals. The certificate is not built for a source with a non-finite or
// huge coordinate, an out-of-range or repeated index within a facet, or more
// than certAdjacentFactor·facets adjacent pairs or certThinLimit thin pairs. A
// check is refused when e ≥ 1/2, when ε' > CertificateTau, when the copy's
// facet indices differ from the source's beyond a reflection, or when any
// copy coordinate is not finite.

// CertificateTau is the largest per-vertex displacement, in millimetres of
// source space, a certificate covers: 2⁻³⁰, about 9.3e-10 mm. A copy near the
// origin rounds by about 1e-13 mm, and a copy 100 m away by below 1e-9 mm.
const CertificateTau = 0x1p-30

// CertificateMinFacets is the facet count from which placements ask for a
// certificate. Smaller meshes scan faster than a source pass costs.
const CertificateMinFacets = 4096

const (
	certThinLimit      = 4096
	certAdjacentFactor = 64
	certCoordLimit     = 0x1p200
	// certRelErr is the relative error charged to each dot product, far above
	// the 6u of its true error bound.
	certRelErr = 1e-12
	// certPertScale is 2τ with a 1e-9 relative cushion, the factor of |n|₁
	// that a gap must exceed.
	certPertScale = 2 * CertificateTau * (1 + 1e-9)
)

// PlacementCertificate is the proof, built once from a source mesh, that its
// rigid copies need only their adjacent and thin pairs checked. It holds a
// private copy of the source, which is also its identity.
type PlacementCertificate struct {
	verts    []r3.Vec
	tris     [][3]int
	adjacent [][2]int32
	thin     [][2]int32
	hits     atomic.Int64
}

// Hits is the number of copies Check has certified.
func (c *PlacementCertificate) Hits() int { return int(c.hits.Load()) }

// ErrNoCertificate is the answer when a mesh has no certificate: the source
// cannot be certified (see the refusals above), is not yet asked for the second
// time, or another goroutine is building it. The caller then scans the copy.
var ErrNoCertificate = errors.New("meshbool: no placement certificate")

// BuildPlacementCertificate certifies the source mesh, or returns
// ErrNoCertificate when it cannot be certified. verts and tris are read, never
// written, and copied into the certificate.
func BuildPlacementCertificate(ctx context.Context, verts []r3.Vec, tris [][3]int) (*PlacementCertificate, error) {
	if !certSourceAdmissible(verts, tris) {
		return nil, ErrNoCertificate
	}
	adjacent := certAdjacentPairs(verts, tris)
	if adjacent == nil {
		return nil, ErrNoCertificate
	}
	thin, ok, err := certThinPairs(ctx, verts, tris)
	if err != nil {
		return nil, err
	}
	if !ok {
		return nil, ErrNoCertificate
	}
	return &PlacementCertificate{verts: slices.Clone(verts), tris: slices.Clone(tris), adjacent: adjacent, thin: thin}, nil
}

func certSourceAdmissible(verts []r3.Vec, tris [][3]int) bool {
	if len(tris) == 0 || len(tris) >= 1<<28 || len(verts) >= 1<<28 {
		return false
	}
	for _, v := range verts {
		if !(math.Abs(v.X) <= certCoordLimit && math.Abs(v.Y) <= certCoordLimit && math.Abs(v.Z) <= certCoordLimit) {
			return false
		}
	}
	for _, t := range tris {
		for _, i := range t {
			if i < 0 || i >= len(verts) {
				return false
			}
		}
		if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
			return false
		}
	}
	return true
}

// certAdjacentPairs lists every pair of facets sharing at least one vertex
// index once, as (low, high) facet indices ascending. A pair is emitted at the
// smallest index it shares. It returns nil when the pairs would exceed the
// limit.
func certAdjacentPairs(verts []r3.Vec, tris [][3]int) [][2]int32 {
	start := make([]int32, len(verts)+1)
	for _, t := range tris {
		for _, v := range t {
			start[v+1]++
		}
	}
	total := 0
	for v := range verts {
		k := int(start[v+1])
		total += k * (k - 1) / 2
		start[v+1] += start[v]
	}
	if total > certAdjacentFactor*len(tris) {
		return nil
	}
	fill := slices.Clone(start[:len(verts)])
	inc := make([]int32, 3*len(tris))
	for i, t := range tris {
		for _, v := range t {
			inc[fill[v]] = int32(i)
			fill[v]++
		}
	}
	out := make([][2]int32, 0, total)
	for v := range verts {
		list := inc[start[v]:start[v+1]]
		for a, fa := range list {
			for _, fb := range list[a+1:] {
				if certMinShared(tris[fa], tris[fb]) == v {
					out = append(out, [2]int32{fa, fb})
				}
			}
		}
	}
	return out
}

// certMinShared is the smallest vertex index two facets share, or -1.
func certMinShared(t, u [3]int) int {
	best := -1
	for _, a := range t {
		if (a == u[0] || a == u[1] || a == u[2]) && (best < 0 || a < best) {
			best = a
		}
	}
	return best
}

// certPaddedBoxes is every facet's box padded by 2τ on every side, each end
// rounded outward one more float so the padding is never rounded down.
func certPaddedBoxes(verts []r3.Vec, tris [][3]int) [][2]r3.Vec {
	const pad = 2 * CertificateTau
	boxes := make([][2]r3.Vec, len(tris))
	down := func(x float64) float64 { return math.Nextafter(x-pad, math.Inf(-1)) }
	up := func(x float64) float64 { return math.Nextafter(x+pad, math.Inf(1)) }
	for i, t := range tris {
		b := TriBox(verts, t)
		boxes[i] = [2]r3.Vec{
			{X: down(b[0].X), Y: down(b[0].Y), Z: down(b[0].Z)},
			{X: up(b[1].X), Y: up(b[1].Y), Z: up(b[1].Z)},
		}
	}
	return boxes
}

// certThinPairs walks every pair of non-adjacent facets whose padded boxes
// overlap and returns, ascending, the pairs the separation test cannot
// certify. ok is false when there are more than certThinLimit of them.
func certThinPairs(ctx context.Context, verts []r3.Vec, tris [][3]int) ([][2]int32, bool, error) {
	tree := newBoxTree(certPaddedBoxes(verts, tris))
	workers := min(runtime.GOMAXPROCS(0), 4)
	tasks, err := tree.selfPairTasks(ctx, 16*workers)
	if err != nil {
		return nil, false, err
	}
	var next, thinCount atomic.Int64
	results := make([][][2]int32, workers)
	errs := make([]error, workers)
	var group sync.WaitGroup
	group.Add(workers)
	for w := range workers {
		go func() {
			defer group.Done()
			visit := func(i, j int) bool {
				if certSeparated(certCorners(verts, tris[i]), certCorners(verts, tris[j])) {
					return true
				}
				results[w] = append(results[w], [2]int32{int32(min(i, j)), int32(max(i, j))})
				return thinCount.Add(1) <= certThinLimit
			}
			for {
				k := int(next.Add(1)) - 1
				if k >= len(tasks) || thinCount.Load() > certThinLimit {
					return
				}
				errs[w] = certScanTask(ctx, tris, &tree, tasks[k], visit)
				if errs[w] != nil {
					return
				}
			}
		}()
	}
	group.Wait()
	for _, err := range errs {
		if err != nil {
			return nil, false, err
		}
	}
	if err := ctx.Err(); err != nil {
		return nil, false, err
	}
	if thinCount.Load() > certThinLimit {
		return nil, false, nil
	}
	var thin [][2]int32
	for _, r := range results {
		thin = append(thin, r...)
	}
	slices.SortFunc(thin, func(a, b [2]int32) int {
		if a[0] != b[0] {
			return int(a[0]) - int(b[0])
		}
		return int(a[1]) - int(b[1])
	})
	return thin, true, nil
}

// certScanTask calls visit for every pair of facets under one node pair whose
// padded boxes overlap and that share no vertex index, in the shape of
// cleanSelfPairs: a node pair whose boxes do not overlap holds no such pair,
// and a leaf pair tests each facet pair's padded boxes. visit returning false
// stops the task.
func certScanTask(ctx context.Context, tris [][3]int, tree *heldBoxTree, task heldNodePair,
	visit func(i, j int) bool) error {
	nodes, facets, slotBoxes := tree.nodes, tree.facets, tree.slotBoxes
	stack := append(make([]heldNodePair, 0, 64), task)
	work := 0
	for len(stack) > 0 {
		last := len(stack) - 1
		pair := stack[last]
		stack = stack[:last]
		a, b := nodes[pair.a], nodes[pair.b]
		if !BoxesOverlap(a.box, b.box) {
			continue
		}
		if a.leafStart < 0 || b.leafStart < 0 {
			children, n := tree.splitSelfPair(pair)
			stack = append(stack, children[:n]...)
			continue
		}
		self := pair.a == pair.b
		end := b.leafStart + b.leafCount
		for s := a.leafStart; s < a.leafStart+a.leafCount; s++ {
			box := slotBoxes[s]
			i := facets[s]
			start := b.leafStart
			if self {
				start = s + 1
			}
			for t := start; t < end; t++ {
				work++
				if work%1024 == 0 {
					if err := ctx.Err(); err != nil {
						return err
					}
				}
				if !BoxesOverlap(box, slotBoxes[t]) {
					continue
				}
				j := facets[t]
				if sharesCorner(tris[i], tris[j]) {
					continue
				}
				if !visit(i, j) {
					return nil
				}
			}
		}
	}
	return ctx.Err()
}

func certCorners(verts []r3.Vec, t [3]int) [3]r3.Vec {
	return [3]r3.Vec{verts[t[0]], verts[t[1]], verts[t[2]]}
}

// certScaleOK reports that x is zero or has a magnitude in [lo, hi]. NaN and
// infinities are not.
func certScaleOK(x, lo, hi float64) bool {
	m := math.Abs(x)
	return x == 0 || m >= lo && m <= hi
}

// certSeparated reports whether the triangles a and b, which share no vertex
// index, are certified apart: some axis built from their edges separates them
// by more than the 2τ|n|₁ of Lemma 2, plus the float error of the test.
func certSeparated(a, b [3]r3.Vec) bool {
	o := a[0]
	d := [6]r3.Vec{{}, a[1].Sub(o), a[2].Sub(o), b[0].Sub(o), b[1].Sub(o), b[2].Sub(o)}
	for _, p := range d {
		if !certScaleOK(p.X, 0x1p-300, 0x1p250) || !certScaleOK(p.Y, 0x1p-300, 0x1p250) ||
			!certScaleOK(p.Z, 0x1p-300, 0x1p250) {
			return false
		}
	}
	ea := [3]r3.Vec{d[1], d[2].Sub(d[1]), d[0].Sub(d[2])}
	eb := [3]r3.Vec{d[4].Sub(d[3]), d[5].Sub(d[4]), d[3].Sub(d[5])}
	na, nb := ea[0].Cross(ea[1]), eb[0].Cross(eb[1])
	if certAxis(&d, na) || certAxis(&d, nb) {
		return true
	}
	for k := range ea {
		if certAxis(&d, na.Cross(ea[k])) || certAxis(&d, nb.Cross(eb[k])) {
			return true
		}
	}
	for _, x := range ea {
		for _, y := range eb {
			if certAxis(&d, x.Cross(y)) {
				return true
			}
		}
	}
	return false
}

// certAxis tests the direction n against the six corner offsets d (the first
// three the triangle a, the last three b, all from a's first corner): the gap
// the axis certifies must exceed the perturbation threshold.
func certAxis(d *[6]r3.Vec, n r3.Vec) bool {
	gap, threshold := certMargin(d, n)
	return gap > threshold
}

// certMargin is the gap the six offsets leave along n, each dot product moved
// by its float error toward the other triangle, and the perturbation 2τ|n|₁
// with the cushion that pays for the rounding of the threshold itself. It
// returns NaN for both when n or an offset is outside the guarded range, so
// that every comparison of them fails.
func certMargin(d *[6]r3.Vec, n r3.Vec) (float64, float64) {
	if !certScaleOK(n.X, 0x1p-400, 0x1p400) || !certScaleOK(n.Y, 0x1p-400, 0x1p400) ||
		!certScaleOK(n.Z, 0x1p-400, 0x1p400) {
		return math.NaN(), math.NaN()
	}
	ax, ay, az := math.Abs(n.X), math.Abs(n.Y), math.Abs(n.Z)
	threshold := (ax + ay + az) * (1 + certRelErr) * certPertScale
	loA, hiA := math.Inf(1), math.Inf(-1)
	loB, hiB := math.Inf(1), math.Inf(-1)
	for k := range 6 {
		p := d[k]
		dot := n.X*p.X + n.Y*p.Y + n.Z*p.Z
		err := certRelErr * (ax*math.Abs(p.X) + ay*math.Abs(p.Y) + az*math.Abs(p.Z))
		lo, hi := dot-err, dot+err
		if k < 3 {
			loA, hiA = min(loA, lo), max(hiA, hi)
		} else {
			loB, hiB = min(loB, lo), max(hiB, hi)
		}
	}
	return max(loB-hiA, loA-hiB), threshold
}

// Check reports whether the copy held, with facets tris, is embedded because
// it is a rigid copy of the certified source. basis is the copy's linear map
// D, whose orthonormality defect it computes exactly, and maxAllow the largest
// bound on a vertex's distance from its exact image |W_i − X_i| the placement
// charged (proofbound.RigidRoundAllow). It returns false, never an error, for
// every refusal listed above; the caller scans the copy then. A canceled
// context returns its error.
func (c *PlacementCertificate) Check(ctx context.Context, basis r3.Basis, maxAllow float64,
	held []r3.Vec, tris [][3]int) (bool, error) {
	if c == nil || !c.sameFacets(tris) || len(held) != len(c.verts) {
		return false, nil
	}
	eps, ok := certEpsilon(basis, maxAllow)
	if !ok || !(eps <= CertificateTau) {
		return false, nil
	}
	for _, v := range held {
		if proofbound.IsNonFinite(v.X) || proofbound.IsNonFinite(v.Y) || proofbound.IsNonFinite(v.Z) {
			return false, nil
		}
	}
	e := newHeldEmbeddingScan(HeldRounding{Verts: held, Tris: tris}, proofbound.NewWorkBudget(ctx), false)
	for i := range tris {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		if !e.facetOpenFloat(i) && zeroXpt(e.normal(i)) {
			return false, nil
		}
	}
	for _, list := range [2][][2]int32{c.adjacent, c.thin} {
		for k, p := range list {
			if k%256 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			if !e.pairEmbedded(int(p[0]), int(p[1])) {
				return false, nil
			}
		}
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	c.hits.Add(1)
	return true, nil
}

// sameFacets reports that tris is the source's facet list, each facet either
// as the source has it or with its last two corners swapped (a reflection).
func (c *PlacementCertificate) sameFacets(tris [][3]int) bool {
	if len(tris) != len(c.tris) {
		return false
	}
	for i, t := range tris {
		s := c.tris[i]
		if t != s && t != [3]int{s[0], s[2], s[1]} {
			return false
		}
	}
	return true
}

// sameSource reports that the mesh is the certified source, vertex for vertex
// and facet for facet.
func (c *PlacementCertificate) sameSource(verts []r3.Vec, tris [][3]int) bool {
	return slices.Equal(verts, c.verts) && slices.Equal(tris, c.tris)
}

// certEpsilon is ε' = (1+e)·maxAllow rounded up, for e the orthonormality
// defect of basis read exactly. ok is false when e is not below 1/2 (Lemma 1
// needs it, and (1+e) bounds 1/√(1−e) only up to 0.618) or a basis entry is
// not finite.
func certEpsilon(basis r3.Basis, maxAllow float64) (float64, bool) {
	e, ok := certDefect(basis)
	if !ok || !(maxAllow >= 0) {
		return 0, false
	}
	return proofbound.ProductUpper(maxAllow, proofbound.AbsSumUpper(1, e)), true
}

// certDefect is the entrywise absolute sum of DᵀD − I for D with columns
// EX, EY, EZ, computed over exact rationals and rounded up. ok is false for a
// non-finite entry or a defect of 1/2 or more.
func certDefect(basis r3.Basis) (float64, bool) {
	cols := [3][3]*big.Rat{}
	for k, c := range [3]r3.Vec{basis.EX, basis.EY, basis.EZ} {
		for m, x := range [3]float64{c.X, c.Y, c.Z} {
			r := proof.FloatRat(x)
			if r == nil {
				return 0, false
			}
			cols[k][m] = r
		}
	}
	defect := new(big.Rat)
	one := big.NewRat(1, 1)
	for i := range cols {
		for j := range cols {
			dot := new(big.Rat)
			for m := range 3 {
				dot.Add(dot, new(big.Rat).Mul(cols[i][m], cols[j][m]))
			}
			if i == j {
				dot.Sub(dot, one)
			}
			defect.Add(defect, dot.Abs(dot))
		}
	}
	if defect.Cmp(big.NewRat(1, 2)) >= 0 {
		return 0, false
	}
	return proofbound.RatFloatUp(defect), true
}

// PlacementCertificates is a goroutine-safe cache of certificates for the
// meshes placed under one context, keyed by content: a mesh with the same
// vertices and facets reuses its certificate whichever slices carry it. A
// certificate is built on the second request for a mesh, so a mesh placed once
// pays no source pass.
type PlacementCertificates struct {
	mu      sync.Mutex
	entries map[uint64][]*certEntry
	built   int
}

type certEntry struct {
	cert *PlacementCertificate
	// state is certSeen after one request, certBuilding while a goroutine
	// builds, and certDone once a build ended, with or without a certificate.
	state int
}

const (
	certSeen = iota
	certBuilding
	certDone
)

type certCacheKey struct{}

// WithPlacementCertificates returns ctx carrying a certificate cache, or ctx
// itself when it already carries one.
func WithPlacementCertificates(ctx context.Context) context.Context {
	if PlacementCertificatesFrom(ctx) != nil {
		return ctx
	}
	return context.WithValue(ctx, certCacheKey{}, &PlacementCertificates{entries: map[uint64][]*certEntry{}})
}

// PlacementCertificatesFrom returns the cache ctx carries, or nil.
func PlacementCertificatesFrom(ctx context.Context) *PlacementCertificates {
	cache, _ := ctx.Value(certCacheKey{}).(*PlacementCertificates)
	return cache
}

// Built is the number of certificates built (or refused after a build) so far.
func (c *PlacementCertificates) Built() int {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.built
}

// Certificate returns the certificate of the mesh, or ErrNoCertificate. The
// first request for a mesh, a request that arrives while another goroutine
// builds it, and a mesh that cannot be certified all return ErrNoCertificate.
// The second request builds from its own slices.
func (c *PlacementCertificates) Certificate(ctx context.Context, verts []r3.Vec, tris [][3]int) (*PlacementCertificate, error) {
	h := certMeshHash(verts, tris)
	c.mu.Lock()
	known := slices.Clone(c.entries[h])
	c.mu.Unlock()
	for _, e := range known {
		// A certificate is immutable once its entry holds it, and the entry is
		// written only under the lock, so read it there.
		c.mu.Lock()
		cert := e.cert
		c.mu.Unlock()
		if cert != nil && cert.sameSource(verts, tris) {
			return cert, nil
		}
	}
	c.mu.Lock()
	var pending *certEntry
	for _, e := range c.entries[h] {
		if e.cert == nil {
			pending = e
			break
		}
	}
	switch {
	case pending == nil:
		c.entries[h] = append(c.entries[h], &certEntry{state: certSeen})
		c.mu.Unlock()
		return nil, ErrNoCertificate
	case pending.state != certSeen:
		c.mu.Unlock()
		return nil, ErrNoCertificate
	}
	pending.state = certBuilding
	c.built++
	c.mu.Unlock()
	cert, err := BuildPlacementCertificate(ctx, verts, tris)
	c.mu.Lock()
	pending.state = certDone
	pending.cert = cert
	c.mu.Unlock()
	return cert, err
}

// certMeshHash mixes every vertex bit and facet index; equal content hashes
// equal. The hash only selects candidates, sameSource decides identity.
func certMeshHash(verts []r3.Vec, tris [][3]int) uint64 {
	h := uint64(0x9e3779b97f4a7c15) ^ uint64(len(verts))<<32 ^ uint64(len(tris))
	mix := func(w uint64) {
		h = (h ^ w) * 0x100000001b3
		h ^= h >> 29
	}
	for _, v := range verts {
		mix(math.Float64bits(v.X + 0))
		mix(math.Float64bits(v.Y + 0))
		mix(math.Float64bits(v.Z + 0))
	}
	for _, t := range tris {
		mix(uint64(t[0])<<42 ^ uint64(t[1])<<21 ^ uint64(t[2]))
	}
	return h
}
