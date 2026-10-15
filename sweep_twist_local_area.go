package decad

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/twistsweep"
)

// twistMixedAreaSlackLocal keeps the existing positive cap charge. Its wall
// charge is the sum of local absolute density gaps, which remains valid when
// Boolean keeps any subset of the held walls. Every live source role and held
// face still has to exist; a smaller number cannot hide a dropped face.
func twistMixedAreaSlackLocal(profile profileRecord, poly twistsweep.TwistPolygon, body *Body,
	held map[*topo.Face]proofbound.RatInterval, wallBound float64) (float64, error) {
	if proofbound.IsNonFinite(wallBound) || wallBound < 0 {
		return 0, fmt.Errorf(`%w: mixed twisted wall has no finite area allowance`, ErrUnsupported)
	}
	byRole := make(map[string]*Face, len(body.Faces()))
	for _, face := range body.Faces() {
		if len(face.n.Origins) != 1 {
			return 0, fmt.Errorf(`%w: a mixed twisted face lost its source role`, ErrUnsupported)
		}
		role := face.n.Origins[0].Role
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
		area, ok := held[faceNode(face)]
		if face == nil || !ok || area.Lo.Sign() < 0 {
			return 0, fmt.Errorf(`%w: mixed twisted wall %s has no held area`, ErrUnsupported, role)
		}
	}
	capCharge := new(big.Rat)
	for _, role := range []string{provenance.RoleCapStart, provenance.RoleCapEnd} {
		face := byRole[role]
		area, ok := held[faceNode(face)]
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
		if len(held.Segment) != 1 || len(held.TwistAreaCells) != 1 ||
			len(held.Segment[0]) != len(held.TwistAreaCells[0]) {
			return 0, fmt.Errorf(`%w: twisted wall lost authenticated cell claims`, ErrUnsupported)
		}
		n := len(held.Segment[0])
		if n == 0 || len(held.Verts) != 2*n || held.Walls != 2*n ||
			len(held.Tris) < held.Walls || len(held.Cell) != held.Walls || len(held.Side) != held.Walls {
			return 0, fmt.Errorf(`%w: twisted wall lost its held cell table`, ErrUnsupported)
		}
		vertexDelta := proofbound.AbsSumUpper(held.Delta,
			math.Max(stations[i].PointError, stations[i+1].PointError))
		for j := range n {
			if held.Segment[0][j] < 0 || held.Segment[0][j] >= len(poly.Points) ||
				held.Cell[2*j] != [2]int{0, j} || held.Cell[2*j+1] != [2]int{0, j} ||
				held.Side[2*j] != 0 || held.Side[2*j+1] != 1 {
				return 0, fmt.Errorf(`%w: twisted wall cell has no source/facet correspondence`, ErrUnsupported)
			}
			jn := (j + 1) % n
			if !sameTwistFacetCorners(held.Tris[2*j], [3]int{j, jn, n + jn}) ||
				!sameTwistFacetCorners(held.Tris[2*j+1], [3]int{j, n + jn, n + j}) {
				return 0, fmt.Errorf(`%w: twisted wall facet changed its cell corners`, ErrUnsupported)
			}
			allow := twistsweep.TwistWallCellAreaL1(held.Verts[j], held.Verts[jn],
				held.Verts[n+j], held.Verts[n+jn], held.TwistAreaCells[0][j],
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
