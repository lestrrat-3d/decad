package decad

import (
	"fmt"
	"math"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
	"github.com/lestrrat-3d/r3"
)

// twistMixedAreaSlackLocal keeps the existing positive cap charge. Its wall
// charge is the sum of local absolute density gaps, which remains valid when
// Boolean keeps any subset of the held walls. Every live source role and held
// face still has to exist; a smaller number cannot hide a dropped face.
func twistMixedAreaSlackLocal(profile profileRecord, poly twistsweep.TwistPolygon, body *Body,
	held map[*Face]proofbound.RatInterval, wallBound float64) (float64, error) {
	if proofbound.IsNonFinite(wallBound) || wallBound < 0 {
		return 0, fmt.Errorf(`%w: mixed twisted wall has no finite area allowance`, ErrUnsupported)
	}
	byRole := make(map[string]*Face, len(body.Faces()))
	for _, face := range body.Faces() {
		if len(face.origins) != 1 {
			return 0, fmt.Errorf(`%w: a mixed twisted face lost its source role`, ErrUnsupported)
		}
		role := face.origins[0].Role
		if byRole[role] != nil {
			return 0, fmt.Errorf(`%w: a mixed twisted source role has two faces`, ErrUnsupported)
		}
		byRole[role] = face
	}
	if len(byRole) != len(profile.Outer.Segments)+2 || len(held) != len(byRole) {
		return 0, fmt.Errorf(`%w: a mixed twisted section has incomplete face roles`, ErrUnsupported)
	}
	for i := range profile.Outer.Segments {
		role := fmt.Sprintf("side(0,0,%d)", i)
		face := byRole[role]
		area, ok := held[face]
		if face == nil || !ok || area.Lo.Sign() < 0 {
			return 0, fmt.Errorf(`%w: mixed twisted wall %s has no held area`, ErrUnsupported, role)
		}
	}
	capCharge := new(big.Rat)
	for _, role := range []string{roleCapStart, roleCapEnd} {
		face := byRole[role]
		area, ok := held[face]
		if face == nil || !ok || area.Lo.Sign() < 0 {
			return 0, fmt.Errorf(`%w: mixed twisted cap %s has no held area`, ErrUnsupported, role)
		}
		capCharge.Add(capCharge, poly.AreaInterval.Hi)
		capCharge.Add(capCharge, area.Hi)
	}
	return proofbound.AbsSumUpper(wallBound, proofbound.RatFloatUp(capCharge)), nil
}

// twistMixedWallAreaL1 sums every authenticated wall cell once. The private
// Loft triangles are checked against their stored cell and side indices; a
// face-level area reading alone cannot bind a source parameter to a facet.
func twistMixedWallAreaL1(cells []loftPayload, stations []straightTwistStation,
	poly twistsweep.TwistPolygon, angle, height float64) (float64, error) {
	if len(cells) < 2 || len(stations) != len(cells)+1 || height <= 0 ||
		proofbound.IsNonFinite(height) || proofbound.IsNonFinite(angle) || angle < 0 {
		return 0, fmt.Errorf(`%w: twisted wall has no source station map`, ErrUnsupported)
	}
	perAngle := proofbound.DivUpper(angle, float64(len(cells)))
	perHeight := proofbound.DivUpper(height, float64(len(cells)))
	sum := 0.0
	for i, held := range cells {
		if len(held.segment) != 1 || len(held.twistAreaCells) != 1 ||
			len(held.segment[0]) != len(held.twistAreaCells[0]) {
			return 0, fmt.Errorf(`%w: twisted wall lost authenticated cell claims`, ErrUnsupported)
		}
		n := len(held.segment[0])
		if n == 0 || len(held.verts) != 2*n || held.walls != 2*n ||
			len(held.tris) < held.walls || len(held.cell) != held.walls || len(held.side) != held.walls {
			return 0, fmt.Errorf(`%w: twisted wall lost its held cell table`, ErrUnsupported)
		}
		vertexDelta := proofbound.AbsSumUpper(held.delta,
			math.Max(stations[i].pointError, stations[i+1].pointError))
		for j := range n {
			if held.segment[0][j] < 0 || held.segment[0][j] >= len(poly.Points) ||
				held.cell[2*j] != [2]int{0, j} || held.cell[2*j+1] != [2]int{0, j} ||
				held.side[2*j] != 0 || held.side[2*j+1] != 1 {
				return 0, fmt.Errorf(`%w: twisted wall cell has no source/facet correspondence`, ErrUnsupported)
			}
			jn := (j + 1) % n
			if !sameTwistFacetCorners(held.tris[2*j], [3]int{j, jn, n + jn}) ||
				!sameTwistFacetCorners(held.tris[2*j+1], [3]int{j, n + jn, n + j}) {
				return 0, fmt.Errorf(`%w: twisted wall facet changed its cell corners`, ErrUnsupported)
			}
			allow := twistWallCellAreaL1(held.verts[j], held.verts[jn],
				held.verts[n+j], held.verts[n+jn], held.twistAreaCells[0][j],
				vertexDelta, poly.MaxRadiusL1, perAngle, perHeight)
			if proofbound.IsNonFinite(allow) {
				return 0, fmt.Errorf(`%w: twisted wall cell has no finite area proof`, ErrUnsupported)
			}
			sum = proofbound.AbsSumUpper(sum, allow)
		}
	}
	if proofbound.IsNonFinite(sum) {
		return 0, fmt.Errorf(`%w: twisted wall area proof overflows`, ErrUnsupported)
	}
	return sum, nil
}

func sameTwistFacetCorners(actual, expected [3]int) bool {
	for _, vertex := range actual {
		matches := false
		for _, target := range expected {
			matches = matches || vertex == target
		}
		if !matches {
			return false
		}
	}
	return actual[0] != actual[1] && actual[1] != actual[2] && actual[2] != actual[0]
}

// twistAreaCellClaims comes from the same authenticated Loft pairing that
// emitted a private Sweep cell's two wall triangles. The circular arm has a
// uniform source parameter on each chord cell; the line arm is affine.
type twistAreaCellClaims struct {
	arcLength float64 // upper bound on integral |source derivative| over s in [0,1]
	energy    float64 // upper bound on integral |source derivative - exact chord|^2
	matched   float64 // upper bound on source curve - exact chord at the same s
}

// twistWallCellAreaL1 bounds the INTEGRAL of the absolute area-density gap
// between a true rotating Line/Arc wall cell and its two held Loft triangles.
// Consequently it bounds the gap on every measurable subset selected by a
// later Boolean, even where positive and negative local errors cancel on the
// complete wall. The four corners are the actual held vertices in Loft's
// (vLo,vHi,wLo,wHi) order. Their source stations are each within vertexDelta.
//
// Let s run along the source cell and u run between the two twist stations.
// The held triangles cover u<=s and s<=u in the unit square. If T is the
// true rotating wall and H is the affine triangle on either half, then
// |area(T)-area(H)| <= |T_s-H_s|*|T_u| + |H_s|*|T_u-H_u| pointwise.
//
// The integral of |T_s-H_s| is at most I + L*a²/8 + 2e + W/3. Here I is
// the authenticated tangent-deviation integral (bounded by sqrt(energy)),
// L is source speed, a is cell turn, e is endpoint displacement, and W is
// the held quad twist vector. The a²/8 term is the linear-interpolation
// remainder for rotation as a function of u. Integrating the affine-versus-
// bilinear triangle split gives W/3 over both halves.
//
// Likewise, integral |T_u-H_u| <= R*a²/2 + a*matched + 2e + W/3.
// The first term bounds derivative-versus-secant rotation; the second uses
// |R1-R0|<=a on the parameter-matched curve/chord departure. The remaining
// terms are the same endpoint and diagonal-split obligations. |T_u| is at
// most height+a*R, while |H_s| is at most the larger held section chord.
// All scalars below are rounded outward; invalid caller claims return +Inf.
func twistWallCellAreaL1(vLo, vHi, wLo, wHi r3.Vec,
	claim twistAreaCellClaims, vertexDelta, radius, angle, height float64) float64 {
	for _, x := range []float64{claim.arcLength, claim.matched, vertexDelta, radius, angle, height} {
		if proofbound.IsNonFinite(x) || x < 0 {
			return math.Inf(1)
		}
	}
	if math.IsNaN(claim.energy) || claim.energy < 0 ||
		!proofbound.FiniteVec(vLo) || !proofbound.FiniteVec(vHi) ||
		!proofbound.FiniteVec(wLo) || !proofbound.FiniteVec(wHi) {
		return math.Inf(1)
	}
	dv := proofbound.HeldDelta(vHi, vLo)
	dw := proofbound.HeldDelta(wHi, wLo)
	chordV, chordW := proofbound.DvLenUpper(dv), proofbound.DvLenUpper(dw)
	maxChord := math.Max(chordV, chordW)
	// A held chord may differ from its exact source chord by two endpoint
	// displacements. The source arc-length claim must still reach it.
	if !proofbound.DvLenAtLeast(proofbound.AbsSumUpper(claim.arcLength,
		proofbound.ProductUpper(2, vertexDelta)), dv) ||
		!proofbound.DvLenAtLeast(proofbound.AbsSumUpper(claim.arcLength,
			proofbound.ProductUpper(2, vertexDelta)), dw) {
		return math.Inf(1)
	}
	twist := proofbound.DvLenUpper(proofarith.DvSub(dw, dv))
	freeI := proofbound.AbsSumUpper(claim.arcLength, maxChord,
		proofbound.ProductUpper(2, vertexDelta))
	i := freeI
	if !proofbound.IsNonFinite(claim.energy) {
		i = math.Min(freeI, proofbound.UpRound(math.Sqrt(claim.energy)))
	}
	a2 := proofbound.ProductUpper(angle, angle)
	twoE := proofbound.ProductUpper(2, vertexDelta)
	warp := proofbound.DivUpper(twist, 3)
	sGap := proofbound.AbsSumUpper(i,
		proofbound.DivUpper(proofbound.ProductUpper(claim.arcLength, a2), 8), twoE, warp)
	uGap := proofbound.AbsSumUpper(
		proofbound.DivUpper(proofbound.ProductUpper(radius, a2), 2),
		proofbound.ProductUpper(angle, claim.matched), twoE, warp)
	uSpeed := proofbound.AbsSumUpper(height, proofbound.ProductUpper(angle, radius))
	return proofbound.AbsSumUpper(proofbound.ProductUpper(uSpeed, sGap),
		proofbound.ProductUpper(maxChord, uGap))
}
