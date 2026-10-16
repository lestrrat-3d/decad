package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"
	"sort"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

const (
	oldObliqueBossAngularCells = 128
	oldObliqueBossBlendCells   = 16
	oldObliqueBossAuditScans   = 16_000_000
)

type oldObliqueBossSample struct {
	u      float64
	corner int
}

type oldObliqueBossMesh struct {
	verts  []r3.Vec
	tris   [][3]int
	src    []int
	groups []facetGroup
}

func (m *oldObliqueBossMesh) point(p r3.Vec) int {
	i := len(m.verts)
	m.verts = append(m.verts, p)
	return i
}

func (m *oldObliqueBossMesh) triangle(a, b, c, source int) {
	m.tris = append(m.tris, [3]int{a, b, c})
	m.src = append(m.src, source)
}

func (m *oldObliqueBossMesh) quad(a, b, c, d, source int) {
	m.triangle(a, b, c, source)
	m.triangle(a, c, d, source)
}

func oldObliqueBossFaceGroup(body *Body, role string) (facetGroup, bool) {
	for _, face := range body.Faces() {
		for _, origin := range face.originRefs() {
			if origin.Role == role && origin.Producer == body.n.Origin.Producer {
				return facetGroup{Origins: face.originRefs(), Planar: face.isPlanar()}, true
			}
		}
	}
	return facetGroup{}, false
}

func oldObliqueBossHasOrigin(face *topo.Face, group facetGroup) bool {
	for _, origin := range face.Origins {
		if slices.Contains(group.Origins, origin) {
			return true
		}
	}
	return false
}
func oldObliqueBossPlateWallRoles(rec *obliqueBossRecord) ([4]string, bool) {
	pp, ok := rec.Plate.Payload.(prismPayload)
	if !ok || len(pp.Profile.Outer.Segments) != 4 {
		return [4]string{}, false
	}
	var roles [4]string // x-low, x-high, y-low, y-high
	for i, segment := range pp.Profile.Outer.Segments {
		line, ok := segment.(lineSeg)
		if !ok {
			return [4]string{}, false
		}
		role := fmt.Sprintf("side(0,%d)", i)
		switch {
		case line.Start.U == rec.Rect.U0 && line.End.U == rec.Rect.U0:
			roles[0] = role
		case line.Start.U == rec.Rect.U1 && line.End.U == rec.Rect.U1:
			roles[1] = role
		case line.Start.V == rec.Rect.V0 && line.End.V == rec.Rect.V0:
			roles[2] = role
		case line.Start.V == rec.Rect.V1 && line.End.V == rec.Rect.V1:
			roles[3] = role
		default:
			return [4]string{}, false
		}
	}
	for _, role := range roles {
		if role == "" {
			return [4]string{}, false
		}
	}
	return roles, true
}
func oldObliqueBossSamples(rec *obliqueBossRecord, radius float64) []oldObliqueBossSample {
	samples := make([]oldObliqueBossSample, 0, oldObliqueBossAngularCells+4)
	for i := range oldObliqueBossAngularCells {
		samples = append(samples, oldObliqueBossSample{u: 2 * math.Pi * float64(i) / oldObliqueBossAngularCells, corner: -1})
	}
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	corners := [4]r3.Vec{
		{X: rec.Rect.U1, Y: rec.Rect.V0},
		{X: rec.Rect.U1, Y: rec.Rect.V1},
		{X: rec.Rect.U0, Y: rec.Rect.V1},
		{X: rec.Rect.U0, Y: rec.Rect.V0},
	}
	sinTilt := math.Hypot(rec.Axis.X, rec.Axis.Y)
	hx, hy := rec.Axis.X/sinTilt, rec.Axis.Y/sinTilt
	for i, corner := range corners {
		dx, dy := corner.X-center.X, corner.Y-center.Y
		u := math.Atan2(-hy*dx+hx*dy, rec.Axis.Z*(hx*dx+hy*dy))
		if u < 0 {
			u += 2 * math.Pi
		}
		samples = append(samples, oldObliqueBossSample{u: u, corner: i})
	}
	sort.Slice(samples, func(i, j int) bool { return samples[i].u < samples[j].u })
	// A square corner can coincide with a regular angular station. Keep the
	// corner so the outer rim changes wall only at an explicit mesh vertex.
	const sameStation = 1e-12
	unique := samples[:0]
	for _, sample := range samples {
		if len(unique) != 0 && sample.u-unique[len(unique)-1].u < sameStation {
			if sample.corner >= 0 {
				unique[len(unique)-1] = sample
			}
			continue
		}
		unique = append(unique, sample)
	}
	if len(unique) > 1 && unique[0].u+2*math.Pi-unique[len(unique)-1].u < sameStation {
		if unique[len(unique)-1].corner >= 0 {
			unique = unique[1:]
		} else {
			unique = unique[:len(unique)-1]
		}
	}
	return unique
}

// oldObliqueBossPlaneRay follows the plate-plane trace of the circle's
// perpendicular section. The same ray sets the rim vertex and corner angle.
func oldObliqueBossPlaneRay(rec *obliqueBossRecord, u float64) (float64, float64) {
	sn, cs := math.Sincos(u)
	d := rec.Ex.Scale(cs).Add(rec.Ey.Scale(sn))
	return d.X - rec.Axis.X*d.Z/rec.Axis.Z,
		d.Y - rec.Axis.Y*d.Z/rec.Axis.Z
}

func oldObliqueBossOuterPoint(rec *obliqueBossRecord, center r3.Vec,
	u float64, corner int) r3.Vec {
	switch corner {
	case 0:
		return r3.Vec{X: rec.Rect.U1, Y: rec.Rect.V0, Z: rec.Top}
	case 1:
		return r3.Vec{X: rec.Rect.U1, Y: rec.Rect.V1, Z: rec.Top}
	case 2:
		return r3.Vec{X: rec.Rect.U0, Y: rec.Rect.V1, Z: rec.Top}
	case 3:
		return r3.Vec{X: rec.Rect.U0, Y: rec.Rect.V0, Z: rec.Top}
	}
	dx, dy := oldObliqueBossPlaneRay(rec, u)
	best := math.Inf(1)
	side := -1
	for i, x := range []float64{rec.Rect.U0, rec.Rect.U1} {
		if dx == 0 {
			continue
		}
		t := (x - center.X) / dx
		if t > 0 && t < best {
			best, side = t, i
		}
	}
	for i, y := range []float64{rec.Rect.V0, rec.Rect.V1} {
		if dy == 0 {
			continue
		}
		t := (y - center.Y) / dy
		if t > 0 && t < best {
			best, side = t, i+2
		}
	}
	p := r3.Vec{X: center.X + best*dx, Y: center.Y + best*dy, Z: rec.Top}
	switch side {
	case 0:
		p.X = rec.Rect.U0
	case 1:
		p.X = rec.Rect.U1
	case 2:
		p.Y = rec.Rect.V0
	case 3:
		p.Y = rec.Rect.V1
	}
	return p
}

func oldBuildObliqueBossFilletMesh(ctx context.Context,
	rec *obliqueBossRecord, radius float64) (*oldObliqueBossMesh, error) {
	m := &oldObliqueBossMesh{}
	wallRoles, ok := oldObliqueBossPlateWallRoles(rec)
	if !ok {
		return nil, fmt.Errorf("%w: the plate walls have no source role map", ErrUnsupported)
	}
	roles := []struct {
		body *Body
		role string
	}{
		{bodyHandle(rec.Plate), provenance.RoleCapStart},
		{bodyHandle(rec.Plate), wallRoles[0]}, {bodyHandle(rec.Plate), wallRoles[1]},
		{bodyHandle(rec.Plate), wallRoles[2]}, {bodyHandle(rec.Plate), wallRoles[3]},
		{bodyHandle(rec.Plate), provenance.RoleCapEnd},
		{bodyHandle(rec.Boss), prismrecord.OuterFirstSideRole}, {bodyHandle(rec.Boss), provenance.RoleCapEnd},
	}
	for _, role := range roles {
		group, ok := oldObliqueBossFaceGroup(role.body, role.role)
		if !ok {
			return nil, fmt.Errorf("%w: a retained oblique boss source has no face %q", ErrUnsupported, role.role)
		}
		m.groups = append(m.groups, group)
	}
	m.groups = append(m.groups, facetGroup{Origins: []provenance.Ref{{Role: "junctionFillet"}}})
	samples := oldObliqueBossSamples(rec, radius)
	n := len(samples)
	outer, lower, plane, side, far := make([]int, n), make([]int, n),
		make([]int, n), make([]int, n), make([]int, n)
	band := make([][]int, oldObliqueBossBlendCells+1)
	for j := range band {
		band[j] = make([]int, n)
	}
	center := rec.Base.Add(rec.Axis.Scale((rec.Top + radius - rec.Base.Z) / rec.Axis.Z))
	for i, sample := range samples {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		sn, cs := math.Sincos(sample.u)
		d := rec.Ex.Scale(cs).Add(rec.Ey.Scale(sn))
		t := (rec.Top + radius - rec.Base.Z - (rec.Radius+radius)*d.Z) / rec.Axis.Z
		c := rec.Base.Add(d.Scale(rec.Radius + radius)).Add(rec.Axis.Scale(t))
		p := c.Sub(r3.Vec{Z: radius})
		p.Z = rec.Top
		s := c.Sub(d.Scale(radius))
		u := rec.Base.Add(d.Scale(rec.Radius)).Add(rec.Axis.Scale(rec.Length))
		o := oldObliqueBossOuterPoint(rec, center, sample.u, sample.corner)
		if !proofbound.FiniteVec(o) || o.X < rec.Rect.U0 || o.X > rec.Rect.U1 ||
			o.Y < rec.Rect.V0 || o.Y > rec.Rect.V1 {
			return nil, fmt.Errorf("%w: an oblique plate rim leaves its source rectangle", ErrUnsupported)
		}
		outer[i], lower[i], plane[i], side[i], far[i] =
			m.point(o), m.point(r3.Vec{X: o.X, Y: o.Y, Z: rec.Bottom}),
			m.point(p), m.point(s), m.point(u)
		band[0][i], band[oldObliqueBossBlendCells][i] = plane[i], side[i]
		for j := 1; j < oldObliqueBossBlendCells; j++ {
			v := float64(j) / oldObliqueBossBlendCells
			q := r3.Vec{Z: 1 - v}.Add(d.Scale(v))
			band[j][i] = m.point(c.Sub(q.Scale(radius / q.Len())))
		}
	}
	bottomCenter := m.point(r3.Vec{X: (rec.Rect.U0 + rec.Rect.U1) / 2,
		Y: (rec.Rect.V0 + rec.Rect.V1) / 2, Z: rec.Bottom})
	farCenter := m.point(rec.Base.Add(rec.Axis.Scale(rec.Length)))
	for i := range samples {
		j := (i + 1) % n
		a, b := m.verts[outer[i]], m.verts[outer[j]]
		wall := -1
		switch {
		case a.X == rec.Rect.U0 && b.X == rec.Rect.U0:
			wall = 1
		case a.X == rec.Rect.U1 && b.X == rec.Rect.U1:
			wall = 2
		case a.Y == rec.Rect.V0 && b.Y == rec.Rect.V0:
			wall = 3
		case a.Y == rec.Rect.V1 && b.Y == rec.Rect.V1:
			wall = 4
		}
		if wall < 0 {
			return nil, fmt.Errorf("%w: an oblique plate rim jumps across a corner", ErrUnsupported)
		}
		m.triangle(bottomCenter, lower[j], lower[i], 0)
		m.quad(lower[i], lower[j], outer[j], outer[i], wall)
		m.quad(outer[i], outer[j], plane[j], plane[i], 5)
		m.quad(side[i], side[j], far[j], far[i], 6)
		m.triangle(farCenter, far[i], far[j], 7)
		for k := range oldObliqueBossBlendCells {
			m.quad(band[k][i], band[k][j], band[k+1][j], band[k+1][i], 8)
		}
	}
	return m, nil
}
func oldObliqueBasisAdmitted(rec *obliqueBossRecord) bool {
	if rec.Axis.Z <= 0.94 {
		return false
	}
	tiltSquared := obliqueboss.ObliqueRatDot(r3.Vec{X: rec.Axis.X, Y: rec.Axis.Y},
		r3.Vec{X: rec.Axis.X, Y: rec.Axis.Y})
	tiltLimit := proofarith.FloatRat(0.349)
	if tiltSquared.Cmp(new(big.Rat).Mul(tiltLimit, tiltLimit)) >= 0 {
		return false
	}
	lo, hi := proofarith.FloatRat(0.999), proofarith.FloatRat(1.009)
	lo.Mul(lo, lo)
	hi.Mul(hi, hi)
	for _, basis := range []r3.Vec{rec.Axis, rec.Ex, rec.Ey} {
		square := obliqueboss.ObliqueRatDot(basis, basis)
		if square.Cmp(lo) <= 0 || square.Cmp(hi) >= 0 {
			return false
		}
	}
	dotLimit := proofarith.FloatRat(0.0009)
	for _, pair := range [][2]r3.Vec{{rec.Axis, rec.Ex}, {rec.Axis, rec.Ey}, {rec.Ex, rec.Ey}} {
		if new(big.Rat).Abs(obliqueboss.ObliqueRatDot(pair[0], pair[1])).Cmp(dotLimit) >= 0 {
			return false
		}
	}
	return true
}

func oldObliqueBossDiameterBound(rec *obliqueBossRecord, boundaryBound float64) float64 {
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.U1), proofarith.FloatRat(rec.Rect.U0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.V1), proofarith.FloatRat(rec.Rect.V0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Top), proofarith.FloatRat(rec.Bottom)))
	w = proofbound.AbsSumUpper(w, proofbound.ProductUpper(2, boundaryBound))
	h = proofbound.AbsSumUpper(h, proofbound.ProductUpper(2, boundaryBound))
	z := proofbound.AbsSumUpper(plateHeight, rec.Length, rec.Radius,
		proofbound.ProductUpper(2, boundaryBound))
	return proofbound.Radius3D(max(w, h, z))
}

// oldObliqueSampleError encloses the exact value of every sampled ideal patch
// point. Trigonometry and normal lengths use rational outward intervals;
// no assumed libm ulp count enters the published boundary bound.
func oldObliqueSampleError(rec *obliqueBossRecord, radius float64,
	u float64, held []r3.Vec) (float64, error) {
	if len(held) != 5+oldObliqueBossBlendCells-1 {
		return 0, fmt.Errorf("%w: an oblique sample has the wrong vertex count", ErrBooleanFailed)
	}
	sn, cs, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(u))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no trigonometric enclosure", ErrUnsupported)
	}
	d := obliqueboss.ObliqueIVVec(rec.Ex).Mul(cs).Add(obliqueboss.ObliqueIVVec(rec.Ey).Mul(sn))
	rPlus := proofbound.IntervalAdd(obliqueboss.ObliqueIVOf(rec.Radius), obliqueboss.ObliqueIVOf(radius))
	topPlus := proofbound.IntervalAdd(obliqueboss.ObliqueIVOf(rec.Top), obliqueboss.ObliqueIVOf(radius))
	tNumerator := proofbound.IntervalSub(
		proofbound.IntervalSub(topPlus, obliqueboss.ObliqueIVOf(rec.Base.Z)),
		proofbound.IntervalMul(rPlus, d.Z))
	t, ok := proofbound.IntervalQuo(tNumerator, obliqueboss.ObliqueIVOf(rec.Axis.Z))
	if !ok {
		return 0, fmt.Errorf("%w: an oblique sample has no axial enclosure", ErrUnsupported)
	}
	c := obliqueboss.ObliqueIVVec(rec.Base).Add(d.Mul(rPlus)).Add(obliqueboss.ObliqueIVVec(rec.Axis).Mul(t))
	vertical := obliqueboss.ObliqueZeroPoint()
	vertical.Z = obliqueboss.ObliqueIVOf(radius)
	p := c.Sub(vertical)
	s := c.Sub(d.Mul(obliqueboss.ObliqueIVOf(radius)))
	far := obliqueboss.ObliqueIVVec(rec.Base).Add(d.Mul(obliqueboss.ObliqueIVOf(rec.Radius))).
		Add(obliqueboss.ObliqueIVVec(rec.Axis).Mul(obliqueboss.ObliqueIVOf(rec.Length)))
	worst := max(p.Error(held[2]), s.Error(held[3]), far.Error(held[4]))
	for j := 1; j < oldObliqueBossBlendCells; j++ {
		v := big.NewRat(int64(j), oldObliqueBossBlendCells)
		q := obliqueboss.ObliqueZeroPoint()
		q.Z = proofbound.PointInterval(new(big.Rat).Sub(big.NewRat(1, 1), v))
		q = q.Add(d.Mul(proofbound.PointInterval(v)))
		n, ok := q.Normalized()
		if !ok {
			return 0, fmt.Errorf("%w: an oblique blend normal has no enclosure", ErrUnsupported)
		}
		point := c.Sub(n.Mul(obliqueboss.ObliqueIVOf(radius)))
		worst = max(worst, point.Error(held[4+j]))
	}
	return worst, nil
}

// oldProveObliqueBossFillet combines a cellwise boundary enclosure with a
// conservative occupied-volume enclosure. The volume collar is intentionally
// larger than the actual fillet: it contains *both* source and held changes
// above the original plate; outside it only the cylinder's radial chord
// annulus and the two small cap slabs can differ.
func oldProveObliqueBossFillet(ctx context.Context, rec *obliqueBossRecord,
	radius float64, mesh *oldObliqueBossMesh) (float64, float64, float64, error) {
	samples := oldObliqueBossSamples(rec, radius)
	stride := 5 + oldObliqueBossBlendCells - 1
	if len(mesh.verts) != len(samples)*stride+2 {
		return 0, 0, 0, fmt.Errorf("%w: the oblique mesh and angular plan disagree", ErrBooleanFailed)
	}
	sampleError := 0.0
	for i, sample := range samples {
		if i%16 == 0 {
			if err := ctx.Err(); err != nil {
				return 0, 0, 0, err
			}
		}
		e, err := oldObliqueSampleError(rec, radius, sample.u,
			mesh.verts[i*stride:(i+1)*stride])
		if err != nil {
			return 0, 0, 0, err
		}
		sampleError = max(sampleError, e)
	}
	farCenter := obliqueboss.ObliqueIVVec(rec.Base).Add(obliqueboss.ObliqueIVVec(rec.Axis).Mul(obliqueboss.ObliqueIVOf(rec.Length)))
	sampleError = max(sampleError, farCenter.Error(mesh.verts[len(mesh.verts)-1]))
	if proofbound.IsNonFinite(sampleError) {
		return 0, 0, 0, fmt.Errorf("%w: an oblique sample has no finite position bound", ErrUnsupported)
	}
	// The regular 128-part turn limits every gap; inserted square corners can
	// only split a gap. Check the actual ordered plan with rational differences,
	// including its wrap across the enclosed value of 2π.
	angularGap := 0.0
	for i, sample := range samples {
		if i+1 < len(samples) {
			angularGap = max(angularGap, proofbound.RatFloatUp(new(big.Rat).Sub(
				proofarith.FloatRat(samples[i+1].u), proofarith.FloatRat(sample.u))))
			continue
		}
		wrap := new(big.Rat).Add(
			new(big.Rat).Sub(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper),
				proofarith.FloatRat(sample.u)), proofarith.FloatRat(samples[0].u))
		angularGap = max(angularGap, proofbound.RatFloatUp(wrap))
	}
	if !(angularGap > 0 && angularGap < 0.05) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique angular plan exceeds its chord proof", ErrUnsupported)
	}
	// Admitted tilt and an exact Gram check keep the held canonical basis
	// within 1.01 of orthonormal. These round constants are coarse rational
	// inequalities, not estimates of floating evaluation error.
	if !oldObliqueBasisAdmitted(rec) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique basis exceeds its proof limits", ErrUnsupported)
	}
	const (
		basis         = 1.01
		minimumNormal = 0.56 // sqrt((1 - 0.35 - 0.01)/2) > 0.56
		maximumQV     = 1.67 // sqrt(1.01^2 + 1 + 2*0.35) < 1.67
	)
	a := proofbound.DivUpper(proofbound.ProductUpper(
		proofbound.AbsSumUpper(rec.Radius, radius), basis), 0.93)
	invM := proofbound.DivUpper(1, minimumNormal)
	invM2 := proofbound.ProductUpper(invM, invM)
	twoBasis := proofbound.ProductUpper(2, basis)
	sixBasisSquared := proofbound.ProductUpper(6, proofbound.ProductUpper(basis, basis))
	duu := proofbound.AbsSumUpper(a, proofbound.ProductUpper(radius,
		proofbound.AbsSumUpper(proofbound.ProductUpper(twoBasis, invM),
			proofbound.ProductUpper(sixBasisSquared, invM2))))
	dvv := proofbound.ProductUpper(radius,
		proofbound.ProductUpper(
			proofbound.ProductUpper(6, proofbound.ProductUpper(maximumQV, maximumQV)), invM2))
	duv := proofbound.ProductUpper(radius,
		proofbound.AbsSumUpper(proofbound.ProductUpper(twoBasis, invM),
			proofbound.ProductUpper(proofbound.ProductUpper(6,
				proofbound.ProductUpper(basis, maximumQV)), invM2)))
	vGap := 1.0 / oldObliqueBossBlendCells
	uSquared := proofbound.ProductUpper(angularGap, angularGap)
	vSquared := proofbound.ProductUpper(vGap, vGap)
	blendChord := proofbound.AbsSumUpper(
		proofbound.ProductUpper(duu, proofbound.DivUpper(uSquared, 8)),
		proofbound.ProductUpper(dvv, proofbound.DivUpper(vSquared, 8)),
		proofbound.ProductUpper(duv,
			proofbound.DivUpper(proofbound.ProductUpper(angularGap, vGap), 4)))
	circleChord := proofbound.ProductUpper(proofbound.AbsSumUpper(rec.Radius, radius),
		proofbound.DivUpper(uSquared, 8))
	// Source circle caps certify the entire affine cylinder by ruling between
	// them. Their tiny source-to-ideal displacement is amplified through the
	// plane crossing and rolling-ball normal, whose admitted denominator is
	// at least 0.56 and whose radius is below R/4.
	sourceGap := proofbound.ProductUpper(16, rec.SourceBound)
	// The mesh's cylinder contact uses the retained source's affine radial
	// vector. The smooth blend normal uses its unit direction. The Gram gate
	// bounds the endpoint separation by 0.02r, including both signs of the
	// near-unit radial length.
	normalizationGap := proofbound.ProductUpper(0.02, radius)
	bound := proofbound.AbsSumUpper(sampleError, sourceGap, normalizationGap,
		blendChord, circleChord)
	if proofbound.IsNonFinite(bound) || !(bound < 0.1) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet exceeds the 0.1 mm source boundary proof: %g", ErrUnsupported, bound)
	}
	// Above the plate, the source and held fillets are both inside this full
	// rectangular collar. Charging its entire volume requires no sign or
	// cancellation assumption about the material added by the fillet.
	outerRadius := proofbound.AbsSumUpper(rec.Radius, radius, bound)
	widthX := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.Length, math.Abs(rec.Axis.X)),
		proofbound.DivUpper(proofbound.ProductUpper(2, outerRadius), 0.93))
	widthY := proofbound.AbsSumUpper(proofbound.ProductUpper(rec.Length, math.Abs(rec.Axis.Y)),
		proofbound.DivUpper(proofbound.ProductUpper(2, outerRadius), 0.93))
	collarHeight := proofbound.AbsSumUpper(proofbound.ProductUpper(radius, 1.35),
		proofbound.ProductUpper(2, bound))
	collarVolume := proofbound.ProductUpper(proofbound.ProductUpper(widthX, widthY), collarHeight)
	// Outside the collar the held boss is a chorded cylinder. The full
	// radial annulus over the cylinder's length and two cap slabs cover its
	// occupied-volume difference from the source cylinder.
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	fourPi := proofbound.ProductUpper(4, piUpper)
	twoPi := proofbound.ProductUpper(2, piUpper)
	annulusVolume := proofbound.ProductUpper(
		proofbound.ProductUpper(fourPi, proofbound.ProductUpper(rec.Radius, bound)),
		proofbound.AbsSumUpper(rec.Length, proofbound.ProductUpper(2, bound)))
	capSlabs := proofbound.ProductUpper(proofbound.ProductUpper(twoPi,
		proofbound.ProductUpper(outerRadius, outerRadius)), bound)
	volumeGap := proofbound.AbsSumUpper(collarVolume, annulusVolume, capSlabs)
	// A deliberately broad area allowance bounds each retained source patch
	// and the blend separately; the held mesh's own area is proven by exact
	// facet arithmetic. Their sum bounds the absolute area discrepancy.
	heldArea := facetproof.MeshAreaUpper(mesh.verts, mesh.tris)
	w := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.U1), proofarith.FloatRat(rec.Rect.U0)))
	h := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Rect.V1), proofarith.FloatRat(rec.Rect.V0)))
	plateHeight := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(rec.Top), proofarith.FloatRat(rec.Bottom)))
	plateArea := proofbound.AbsSumUpper(proofbound.ProductUpper(2, proofbound.ProductUpper(w, h)),
		proofbound.ProductUpper(proofbound.ProductUpper(2, proofbound.AbsSumUpper(w, h)),
			plateHeight))
	bossArea := proofbound.AbsSumUpper(
		proofbound.ProductUpper(twoPi, proofbound.ProductUpper(
			proofbound.AbsSumUpper(rec.Radius, bound), proofbound.AbsSumUpper(rec.Length, bound))),
		proofbound.ProductUpper(piUpper, proofbound.ProductUpper(outerRadius, outerRadius)))
	blendArea := proofbound.ProductUpper(twoPi,
		proofbound.ProductUpper(proofbound.AbsSumUpper(a,
			proofbound.ProductUpper(proofbound.ProductUpper(2,
				proofbound.ProductUpper(radius, basis)), invM)),
			proofbound.ProductUpper(proofbound.ProductUpper(2,
				proofbound.ProductUpper(radius, maximumQV)), invM)))
	areaSlack := proofbound.AbsSumUpper(heldArea,
		proofbound.ProductUpper(2, proofbound.AbsSumUpper(plateArea, bossArea, blendArea)))
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return 0, 0, 0, fmt.Errorf("%w: the oblique fillet has no finite volume or area proof", ErrUnsupported)
	}
	return bound, volumeGap, areaSlack, nil
}
