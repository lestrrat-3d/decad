package loftcontact

import (
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
)

// QuarterWallRectangle reads four complete axis-aligned lines as one
// first-quadrant rectangle and names its lower and upper source edges.
func QuarterWallRectangle(budget *proofbound.WorkBudget,
	lines []sectionrecord.LineSeg) ([4]*big.Rat, [2]int, bool, error) {
	var bounds [4]*big.Rat
	var horizontal [2]int
	if len(lines) != 4 {
		return bounds, horizontal, false, nil
	}
	var points [8]sectionrecord.Point2
	for i, line := range lines {
		if err := budget.Step(); err != nil {
			return bounds, horizontal, false, err
		}
		if line.TStart != 0 || line.TEnd != 1 ||
			(line.Start.U != line.End.U && line.Start.V != line.End.V) ||
			line.Start == line.End {
			return bounds, horizontal, false, nil
		}
		points[2*i], points[2*i+1] = line.Start, line.End
	}
	xMin, xMax, yMin, yMax := points[0].U, points[0].U, points[0].V, points[0].V
	for _, p := range points {
		if err := budget.Step(); err != nil {
			return bounds, horizontal, false, err
		}
		if proofbound.IsNonFinite(p.U) || proofbound.IsNonFinite(p.V) {
			return bounds, horizontal, false, nil
		}
		xMin, xMax = min(xMin, p.U), max(xMax, p.U)
		yMin, yMax = min(yMin, p.V), max(yMax, p.V)
	}
	if xMin <= 0 || yMin <= 0 || xMin >= xMax || yMin >= yMax {
		return bounds, horizontal, false, nil
	}
	seen := [4]bool{}
	for i, line := range lines {
		if err := budget.Step(); err != nil {
			return bounds, horizontal, false, err
		}
		a, b := line.Start, line.End
		kind := -1
		switch {
		case a.V == yMin && b.V == yMin &&
			((a.U == xMin && b.U == xMax) || (a.U == xMax && b.U == xMin)):
			kind = 0
		case a.V == yMax && b.V == yMax &&
			((a.U == xMin && b.U == xMax) || (a.U == xMax && b.U == xMin)):
			kind = 1
		case a.U == xMin && b.U == xMin &&
			((a.V == yMin && b.V == yMax) || (a.V == yMax && b.V == yMin)):
			kind = 2
		case a.U == xMax && b.U == xMax &&
			((a.V == yMin && b.V == yMax) || (a.V == yMax && b.V == yMin)):
			kind = 3
		}
		if kind < 0 || seen[kind] {
			return bounds, horizontal, false, nil
		}
		seen[kind] = true
		if kind < 2 {
			horizontal[kind] = i
		}
	}
	for _, found := range seen {
		if !found {
			return bounds, horizontal, false, nil
		}
	}
	return [4]*big.Rat{proofarith.FloatRat(xMin), proofarith.FloatRat(xMax),
		proofarith.FloatRat(yMin), proofarith.FloatRat(yMax)}, horizontal, true, nil
}

// QuarterWallAxisLine checks one complete radial line of the quarter cap.
func QuarterWallAxisLine(line sectionrecord.LineSeg, radius *big.Rat) bool {
	if radius == nil || radius.Sign() <= 0 || line.TStart != 0 || line.TEnd != 1 {
		return false
	}
	r, exact := radius.Float64()
	if !exact {
		return false
	}
	a, b := line.Start, line.End
	if a.U != 0 || a.V != 0 {
		a, b = b, a
	}
	return a.U == 0 && a.V == 0 &&
		(b.U == r && b.V == 0 || b.U == 0 && b.V == r)
}
