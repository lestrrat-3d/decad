package loftcylinder

import (
	"math"
	"math/big"
	"sort"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// maxUnionTeeth is the most separately certified Loft teeth one circular
// Join admits.
const maxUnionTeeth = 32

// Tooth is one admitted axial Loft tooth of a circular Join: its proof
// payload, verified mesh proof, cap angle sectors and seam gate results.
type Tooth struct {
	// Operand indexes this tooth in the caller's operand lists.
	Operand int
	// Loft is the axial proof tooth's payload.
	Loft loftrecord.Payload
	// MeshBound and MeshVolSymDiff are the proof tooth's verified mesh bounds.
	MeshBound      float64
	MeshVolSymDiff float64
	// Bridge is the axial distance from the proof tooth to its original operand.
	Bridge float64
	// UpperSeparationBridge widens only the moving-plane separation proof
	// when a later mirrored assembly translates this tooth's upper source.
	UpperSeparationBridge float64
	// RootStart is the first held station of the root arc on each cap.
	RootStart int
	// Bottom and Top are the start and end angles of each cap's sector,
	// and BottomIV and TopIV enclose their true values.
	Bottom, Top     [2]float64
	BottomIV, TopIV [2]proofbound.RatInterval
	// Vertex maps each cap's held stations 0..RootStart to joined-mesh
	// vertices. Join.Shell sets it.
	Vertex  [2][]int
	RootGap float64
	Inset   float64
	Reach   float64
	Shift   float64
	Angular float64
	ReachOK bool
	Collar  float64
	Wedge   float64
}

// BlankRadius reads the radius of an unplaced exact circular blank: one
// centered CCW circle in the XY frame with no section displacement.
func BlankRadius(pp prismrecord.Payload) (float64, bool) {
	if pp.SurfaceResult || pp.Xform != r3.Identity() ||
		pp.Z0Delta != 0 || pp.Z1Delta != 0 || pp.SectionDelta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.Profile.Outer.Segments) != 1 ||
		pp.Frame.Origin() != (r3.Vec{}) || pp.Frame.U() != (r3.Vec{X: 1}) ||
		pp.Frame.V() != (r3.Vec{Y: 1}) || pp.Z1 <= pp.Z0 {
		return 0, false
	}
	circle, ok := pp.Profile.Outer.Segments[0].(sectionrecord.CircleSeg)
	if !ok || circle.Center != (sectionrecord.Point2{}) || !circle.CCW {
		return 0, false
	}
	radius := circle.Radius.Base()
	if radius <= 0 || proofbound.IsNonFinite(radius) {
		return 0, false
	}
	return radius, true
}

// ToothSections reports whether lp is an authenticated six-curve Loft
// whose two XY sections lie flush on the blank's cap planes z0 and z1.
func ToothSections(lp loftrecord.Payload, z0, z1 float64) bool {
	if !lp.AuthenticatedReconstruction || lp.SurfaceResult ||
		lp.SourceIdentity == nil || len(lp.Profile0.Holes) != 0 ||
		len(lp.Profile1.Holes) != 0 || len(lp.Profile0.Outer.Segments) != 6 ||
		len(lp.Profile1.Outer.Segments) != 6 || len(lp.Segment) != 1 ||
		len(lp.Verts)%2 != 0 || len(lp.Verts) < 16 {
		return false
	}
	return SectionKinds(lp) && lp.Frame0.U() == (r3.Vec{X: 1}) &&
		lp.Frame0.V() == (r3.Vec{Y: 1}) && lp.Frame1.U() == (r3.Vec{X: 1}) &&
		lp.Frame1.V() == (r3.Vec{Y: 1}) &&
		lp.Frame0.Origin() == (r3.Vec{Z: z0}) &&
		lp.Frame1.Origin() == (r3.Vec{Z: z1})
}

// ToothRoot checks an axial proof tooth's held layout and returns its first
// root station. The root arc's stations must be the cap's final contiguous
// run, start at station 2 or later and hold at least two stations.
func ToothRoot(lp loftrecord.Payload) (int, bool) {
	if !AxialPlacement(lp.Xform) {
		return 0, false
	}
	n := len(lp.Verts) / 2
	if len(lp.Segment[0]) != n || lp.Walls != 2*n ||
		len(lp.Tris) != 4*n-4 {
		return 0, false
	}
	root := -1
	for j, segment := range lp.Segment[0] {
		if segment == 5 && root < 0 {
			root = j
		}
		if (root < 0 && segment == 5) || (root >= 0 && segment != 5) {
			return 0, false
		}
	}
	if root < 2 || root >= n-1 {
		return 0, false
	}
	return root, true
}

// Admit sets the tooth's cap angle sectors and runs its seam gates against
// a blank of the given radius between z0 and z1. Loft, RootStart, MeshBound
// and UpperSeparationBridge must already be set.
func (t *Tooth) Admit(radius, z0, z1 float64) bool {
	if t.UpperSeparationBridge < 0 || proofbound.IsNonFinite(t.UpperSeparationBridge) {
		return false
	}
	lp, root := t.Loft, t.RootStart
	n := len(lp.Verts) / 2
	for cap := range 2 {
		offset := cap * n
		p0, p1 := lp.Verts[offset], lp.Verts[offset+root]
		start, end := math.Atan2(p0.Y, p0.X), math.Atan2(p1.Y, p1.X)
		startIV := proofbound.Atan2Interval(proofarith.FloatRat(p0.Y), proofarith.FloatRat(p0.X), false)
		endIV := proofbound.Atan2Interval(proofarith.FloatRat(p1.Y), proofarith.FloatRat(p1.X), false)
		for end <= start {
			end += 2 * math.Pi
			endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
		}
		if end-start <= 0 || end-start >= math.Pi/2 {
			return false
		}
		if cap == 0 {
			t.Bottom = [2]float64{start, end}
			t.BottomIV = [2]proofbound.RatInterval{startIV, endIV}
			continue
		}
		for start-t.Bottom[0] > math.Pi {
			start -= 2 * math.Pi
			end -= 2 * math.Pi
			startIV = proofbound.IntervalSub(startIV, proofbound.TwoPiInterval())
			endIV = proofbound.IntervalSub(endIV, proofbound.TwoPiInterval())
		}
		for start-t.Bottom[0] < -math.Pi {
			start += 2 * math.Pi
			end += 2 * math.Pi
			startIV = proofbound.IntervalAdd(startIV, proofbound.TwoPiInterval())
			endIV = proofbound.IntervalAdd(endIV, proofbound.TwoPiInterval())
		}
		t.Top = [2]float64{start, end}
		t.TopIV = [2]proofbound.RatInterval{startIV, endIV}
	}
	profiles := [2]momentinput.Profile{lp.Profile0, lp.Profile1}
	gap, ok := RootRadiusGap(lp.Xform, profiles, radius)
	if !ok {
		return false
	}
	height := proofbound.RatFloatUp(new(big.Rat).Sub(
		proofarith.FloatRat(z1), proofarith.FloatRat(z0)))
	collar, ok := RootCollar(lp.Xform, profiles, radius, height)
	if !ok {
		return false
	}
	connectorCollar, ok := ConnectorGate(lp.Verts, lp.Tris, lp.Segment, root, radius, height, t.MeshBound)
	if !ok {
		return false
	}
	collar = max(collar, connectorCollar)
	reach, shift, angular, ok := ConnectorReach(lp.Verts, lp.Tris, lp.Segment,
		profiles, lp.Xform.Basis().EX, lp.Proof.FacetDeparture, root, radius, collar)
	if ok {
		circle, err := SourceCircleOf(lp, radius)
		ok = err == nil && circle.OrderedOverSafeInterval()
	}
	reachOK := ok
	inset, ok := RadialGate(lp.Verts, lp.Tris, lp.Segment, root, radius, t.MeshBound, gap)
	if !ok {
		return false
	}
	wedge, ok := WedgeGate(lp.Verts, lp.Tris, root, z0, z1)
	if !ok {
		return false
	}
	t.RootGap, t.Inset = gap, inset
	t.Reach, t.Shift, t.Angular = reach, shift, angular
	t.ReachOK = reachOK
	t.Collar, t.Wedge = collar, wedge
	return true
}

// Separate orders teeth by their bottom sector start and proves every pair
// disjoint and every neighbouring cap sector apart. It refuses no teeth or
// more than maxUnionTeeth.
func Separate(teeth []Tooth) bool {
	if len(teeth) == 0 || len(teeth) > maxUnionTeeth {
		return false
	}
	sort.Slice(teeth, func(i, j int) bool { return teeth[i].Bottom[0] < teeth[j].Bottom[0] })
	// Try a fixed through-axis plane first. A twisted pair may need a plane
	// whose normal changes with height; that case checks whole triangles.
	movingWork := 8_000_000
	for i := range teeth {
		for j := i + 1; j < len(teeth); j++ {
			center := func(part Tooth) (float64, float64) {
				angle := (part.Bottom[0] + part.Bottom[1] +
					part.Top[0] + part.Top[1]) / 4
				return math.Cos(angle), math.Sin(angle)
			}
			ax, ay := center(teeth[i])
			bx, by := center(teeth[j])
			nx, ny := bx-ax, by-ay
			if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
				return false
			}
			rx, ry := proofarith.FloatRat(nx), proofarith.FloatRat(ny)
			length2 := new(big.Rat).Add(new(big.Rat).Mul(rx, rx),
				new(big.Rat).Mul(ry, ry))
			length := proofbound.RatSqrtUp(length2)
			if length <= 0 || proofbound.IsNonFinite(length) {
				return false
			}
			fixed := true
			for k, part := range []Tooth{teeth[i], teeth[j]} {
				margin := proofbound.ProductUpper(length, part.MeshBound)
				marginRat := proofarith.FloatRat(margin)
				for _, p := range part.Loft.Verts {
					dot := new(big.Rat).Add(new(big.Rat).Mul(rx, proofarith.FloatRat(p.X)),
						new(big.Rat).Mul(ry, proofarith.FloatRat(p.Y)))
					if k == 0 && dot.Cmp(new(big.Rat).Neg(marginRat)) >= 0 ||
						k == 1 && dot.Cmp(marginRat) <= 0 {
						fixed = false
						break
					}
				}
				if !fixed {
					break
				}
			}
			if !fixed && !movingSeparate(teeth[i], teeth[j], &movingWork) {
				return false
			}
		}
	}
	for i := range teeth {
		j := (i + 1) % len(teeth)
		for cap := range 2 {
			a, b := teeth[i].Bottom, teeth[j].Bottom
			if cap == 1 {
				a, b = teeth[i].Top, teeth[j].Top
			}
			for b[0] <= a[0] {
				b[0] += 2 * math.Pi
				b[1] += 2 * math.Pi
			}
			if a[1]+0.02 >= b[0] || b[0]-a[1] >= 2*math.Pi {
				return false
			}
		}
	}
	return true
}

// movingSeparate proves two axial Loft teeth are disjoint at every height.
// Its chosen through-axis normal is linear in z. The dot product with a
// triangle is quadratic in barycentric coordinates, so all six degree-two
// Bernstein coefficients must have the required strict sign. The Loft's
// matched wall correspondence preserves z and reaches both ways; its
// boundary departure widens each plane by its maximum normal length. At
// fixed z the plane is linear in XY, so each filled Loft section lies on the
// same side as its entire boundary.
func movingSeparate(a, b Tooth, remaining *int) bool {
	if len(a.Loft.Tris) > (*remaining)/6-len(b.Loft.Tris) {
		return false
	}
	*remaining -= 6 * (len(a.Loft.Tris) + len(b.Loft.Tris))

	center := func(part Tooth, endIndex int) (float64, float64) {
		angles := part.Bottom
		if endIndex == 1 {
			angles = part.Top
		}
		angle := (angles[0] + angles[1]) / 2
		return math.Cos(angle), math.Sin(angle)
	}
	var normals [2][2]*big.Rat
	maxNorm := 0.0
	for cap := range 2 {
		ax, ay := center(a, cap)
		bx, by := center(b, cap)
		nx, ny := bx-ax, by-ay
		if !proofbound.FiniteVec(r3.Vec{X: nx, Y: ny}) {
			return false
		}
		normals[cap] = [2]*big.Rat{proofarith.FloatRat(nx), proofarith.FloatRat(ny)}
		squared := new(big.Rat).Add(
			new(big.Rat).Mul(normals[cap][0], normals[cap][0]),
			new(big.Rat).Mul(normals[cap][1], normals[cap][1]))
		maxNorm = max(maxNorm, proofbound.RatSqrtUp(squared))
	}
	if maxNorm <= 0 || proofbound.IsNonFinite(maxNorm) {
		return false
	}

	z0 := a.Loft.Frame0.Origin().Z
	z1 := a.Loft.Frame1.Origin().Z
	if !(z1 > z0) || b.Loft.Frame0.Origin().Z != z0 || b.Loft.Frame1.Origin().Z != z1 {
		return false
	}
	zStart := proofarith.FloatRat(z0)
	zSpan := new(big.Rat).Sub(proofarith.FloatRat(z1), zStart)
	deltaX := new(big.Rat).Sub(normals[1][0], normals[0][0])
	deltaY := new(big.Rat).Sub(normals[1][1], normals[0][1])
	deltaNorm := proofbound.RatSqrtUp(new(big.Rat).Add(
		new(big.Rat).Mul(deltaX, deltaX), new(big.Rat).Mul(deltaY, deltaY)))
	one := big.NewRat(1, 1)
	two := big.NewRat(2, 1)
	for side, part := range []Tooth{a, b} {
		bound := max(part.MeshBound, part.Loft.Proof.FacetDeparture)
		if bound < 0 || proofbound.IsNonFinite(bound) {
			return false
		}
		margin := proofbound.ProductUpper(maxNorm, bound)
		// A bridged source point differs in height from its proof proxy.
		// A mirrored upper source can add a second constant-height shift.
		// The moving normal changes by at most |N1-N0|*shift/height;
		// charge its dot product with the source's maximum XY radius.
		shift := proofbound.AbsSumUpper(part.Bridge, part.UpperSeparationBridge)
		if shift > 0 {
			radius := 0.0
			for _, p := range part.Loft.Verts {
				q := new(big.Rat).Add(
					new(big.Rat).Mul(proofarith.FloatRat(p.X), proofarith.FloatRat(p.X)),
					new(big.Rat).Mul(proofarith.FloatRat(p.Y), proofarith.FloatRat(p.Y)))
				radius = max(radius, proofbound.RatSqrtUp(q))
			}
			radius = proofbound.AbsSumUpper(radius, part.Loft.Proof.FacetDeparture)
			fraction := proofbound.RatFloatUp(new(big.Rat).Quo(
				proofarith.FloatRat(shift), zSpan))
			margin = proofbound.AbsSumUpper(margin,
				proofbound.ProductUpper(proofbound.ProductUpper(deltaNorm, radius), fraction))
		}
		if proofbound.IsNonFinite(margin) {
			return false
		}
		limit := proofarith.FloatRat(margin)
		if side == 0 {
			limit.Neg(limit)
		}
		for _, tri := range part.Loft.Tris {
			var points [3][2]*big.Rat
			var moving [3][2]*big.Rat
			for corner, index := range tri {
				point := part.Loft.Verts[index]
				if !proofbound.FiniteVec(point) {
					return false
				}
				points[corner] = [2]*big.Rat{proofarith.FloatRat(point.X), proofarith.FloatRat(point.Y)}
				t := new(big.Rat).Quo(
					new(big.Rat).Sub(proofarith.FloatRat(point.Z), zStart), zSpan)
				if t.Sign() < 0 || t.Cmp(one) > 0 {
					return false
				}
				for axis := range 2 {
					moving[corner][axis] = new(big.Rat).Add(normals[0][axis],
						new(big.Rat).Mul(t, new(big.Rat).Sub(normals[1][axis], normals[0][axis])))
				}
			}
			dot := func(i, j int) *big.Rat {
				return new(big.Rat).Add(
					new(big.Rat).Mul(moving[i][0], points[j][0]),
					new(big.Rat).Mul(moving[i][1], points[j][1]))
			}
			for i := range 3 {
				for j := i; j < 3; j++ {
					coefficient := dot(i, j)
					if i != j {
						coefficient.Quo(coefficient.Add(coefficient, dot(j, i)), two)
					}
					if side == 0 && coefficient.Cmp(limit) >= 0 ||
						side == 1 && coefficient.Cmp(limit) <= 0 {
						return false
					}
				}
			}
		}
	}
	return true
}
