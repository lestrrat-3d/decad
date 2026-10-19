package decad

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

const loftVolumeComposerRatBits = 8192
const loftVolumeOutputSteps = 65_536

// loftVolumeProof is transient. Its bound is meaningful only for the source
// partition and boundary proof certified from the same live operands.
type loftVolumeProof struct {
	boundary          loftOutputBoundaryProof
	sourceVolume      *big.Rat
	symDiffUpper      float64
	totalOutputVisits uint64
}

// loftVolumeCompose uses the source partition carried by its boundary proof.
// The payloads must come from the same live records passed to that proof.
func loftVolumeCompose(ctx context.Context, operation *proofbound.WorkBudget,
	pp prismPayload, lp loftPayload, boundary loftOutputBoundaryProof,
) (loftVolumeProof, bool, error) {
	var no loftVolumeProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if operation == nil || boundary.workVisits == 0 ||
		boundary.workVisits >= loftVolumeOutputSteps ||
		len(boundary.shell.triangles) == 0 || len(boundary.shell.triangles) > 512 ||
		len(boundary.shell.vertices) == 0 || len(boundary.shell.vertices) > 512 {
		return no, false, nil
	}
	limit := proofbound.NewWallWorkBudgetWithOperation(
		loftVolumeOutputSteps-boundary.workVisits, operation)
	var volumeVisits uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := limit.Step(); err != nil {
				return err
			}
			volumeVisits++
			return nil
		},
		ErrFn: limit.Err,
	}
	shell := boundary.shell
	if admitted, err := loftVolumeSourceEquality(ctx, budget, pp, lp, boundary.source, shell); err != nil || !admitted {
		return no, false, err
	}
	sourceVolume, admitted, err := loftVolumeExactShell(budget, shell)
	if err != nil || !admitted {
		return no, false, err
	}
	if !loftVolumeRatFits(sourceVolume) || sourceVolume.Sign() <= 0 {
		return no, false, nil
	}
	bound := new(big.Rat)
	for _, tri := range shell.triangles {
		if err := ctx.Err(); err != nil {
			return no, false, err
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		var points [6][3]*big.Rat
		for corner, index := range tri {
			if index < 0 || index >= len(shell.vertices) {
				return no, false, nil
			}
			vertex := shell.vertices[index]
			for axis, coordinate := range vertex.world {
				if !loftVolumeRatFits(coordinate) {
					return no, false, nil
				}
				points[corner][axis] = coordinate
			}
			for axis, coordinate := range [3]float64{vertex.held.X, vertex.held.Y, vertex.held.Z} {
				if proofbound.IsNonFinite(coordinate) {
					return no, false, nil
				}
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				points[corner+3][axis] = proofarith.FloatRat(coordinate)
				if !loftVolumeRatFits(points[corner+3][axis]) {
					return no, false, nil
				}
			}
		}
		term, admitted, err := loftcontact.VolumeHullBound(budget, points)
		if err != nil || !admitted {
			return no, false, err
		}
		if !loftVolumeRatFits(term) || term.Sign() < 0 || !loftVolumeSumFits(bound, term) {
			return no, false, nil
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		bound.Add(bound, term)
		if !loftVolumeRatFits(bound) {
			return no, false, nil
		}
	}
	if err := budget.Step(); err != nil {
		return no, false, err
	}
	upper, rounded := loftVolumeRoundUp(bound)
	if !rounded {
		return no, false, nil
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
		return no, false, nil
	}
	upperRat := proofarith.FloatRat(upper)
	if !loftVolumeCompareFits(sourceVolume, upperRat) || sourceVolume.Cmp(upperRat) <= 0 {
		return no, false, nil
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return loftVolumeProof{
		boundary: boundary, sourceVolume: new(big.Rat).Set(sourceVolume),
		symDiffUpper: upper, totalOutputVisits: boundary.workVisits + volumeVisits,
	}, true, nil
}

func loftVolumeRatFits(value *big.Rat) bool {
	return value != nil && value.Denom().Sign() > 0 &&
		value.Num().BitLen() <= loftVolumeComposerRatBits &&
		value.Denom().BitLen() <= loftVolumeComposerRatBits
}

// loftVolumeRoundUp preflights the integer alignment used by RatFloat64 and
// the cross products used by the exact rounding-direction comparison.
func loftVolumeRoundUp(value *big.Rat) (float64, bool) {
	if !loftVolumeRatFits(value) || value.Sign() < 0 {
		return 0, false
	}
	if value.Sign() == 0 {
		return 0, true
	}
	numeratorBits, denominatorBits := value.Num().BitLen(), value.Denom().BitLen()
	if value.Denom().TrailingZeroBits() != uint(denominatorBits-1) {
		shift := 54 - (numeratorBits - denominatorBits)
		if shift > 0 && numeratorBits+shift > loftVolumeComposerRatBits {
			return 0, false
		}
	}
	nearest, _ := proofarith.RatFloat64(value)
	if proofbound.IsNonFinite(nearest) {
		return 0, false
	}
	nearestRat := proofarith.FloatRat(nearest)
	if !loftVolumeCompareFits(nearestRat, value) {
		return 0, false
	}
	if nearestRat.Cmp(value) < 0 {
		nearest = proofarith.NextUp(nearest)
	}
	return nearest, !proofbound.IsNonFinite(nearest)
}

// Adding a/b and c/d can need ad+bc over bd before reduction. Refuse before
// that allocation if either unreduced intermediate could pass the ceiling.
func loftVolumeSumFits(a, b *big.Rat) bool {
	if !loftVolumeRatFits(a) || !loftVolumeRatFits(b) {
		return false
	}
	if a.Sign() == 0 || b.Sign() == 0 {
		return true
	}
	an, ad := a.Num().BitLen(), a.Denom().BitLen()
	bn, bd := b.Num().BitLen(), b.Denom().BitLen()
	return max(an+bd, bn+ad)+1 <= loftVolumeComposerRatBits &&
		ad+bd <= loftVolumeComposerRatBits
}

func loftVolumeCompareFits(a, b *big.Rat) bool {
	return loftVolumeRatFits(a) && loftVolumeRatFits(b) &&
		a.Num().BitLen()+b.Denom().BitLen() <= loftVolumeComposerRatBits &&
		b.Num().BitLen()+a.Denom().BitLen() <= loftVolumeComposerRatBits
}

const loftVolumeSourceMaxRatBits = 8192

// loftVolumeSourceEquality proves that the exact triangles in shell tile the
// outward boundary of the admitted source union. The caller must pass the
// partition of these live, authenticated line-only records and the same
// output-phase budget used to build shell. This is a private, transient source
// fact; it says nothing about held-shell embedding or a volume bound.
//
// At every interior height below the Loft end, the convex Loft section and
// convex prism intersect at exactly the two ordered carrier seams. All other
// prism halfplanes strictly contain the moving Loft vertices. Thus the
// exposed fixed wall is the whole noncarrier prism boundary plus the carrier
// complement of the Loft interval; the exposed moving wall is precisely the
// Loft boundary outside the prism. Above the Loft end only the prism remains.
// The lower cap is P union L, the middle cap is L minus P, and the upper cap
// is P. The exact cap arrangement and side-interval constructors below tile
// those disjoint patches once. A triangle-by-triangle comparison then proves
// that shell has neither an omitted nor a doubled or internal patch.
func loftVolumeSourceEquality(ctx context.Context, budget *proofbound.WorkBudget,
	pp prismPayload, lp loftPayload, source loftSourcePartition, shell loftOutputShell) (bool, error) {
	arithmetic := &loftVolumeArithmetic{ok: true}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if budget == nil || len(shell.vertices) == 0 || len(shell.vertices) > 512 ||
		len(shell.triangles) == 0 || len(shell.triangles) > 512 ||
		len(shell.faces) != len(shell.triangles) || len(shell.triangleBounds) != len(shell.triangles) ||
		len(source.walls) < 3 || len(source.walls) > 32 ||
		source.capLevels[0] == nil || source.capLevels[1] == nil || source.capLevels[2] == nil {
		return false, nil
	}
	parent := budget
	budget = &proofbound.WorkBudget{
		StepFn: func() error {
			if err := ctx.Err(); err != nil {
				return err
			}
			return parent.Step()
		},
		ErrFn: func() error {
			if err := ctx.Err(); err != nil {
				return err
			}
			return parent.Err()
		},
	}
	if !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil || lp.SurfaceResult ||
		lp.ThroughBoreCut || lp.AnalyticBore || lp.ConstructionDelta != 0 ||
		len(lp.BlendSegs) != 0 || len(lp.Profile0.Holes) != 0 || len(lp.Profile1.Holes) != 0 ||
		pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 ||
		len(pp.Profile.Holes) != 0 || len(pp.BlendSegs) != 0 ||
		pp.Xform != r3.Identity() || lp.Xform != r3.Identity() ||
		pp.Z0 != 0 || pp.Z1 <= 0 || lp.Frame0 != pp.Frame ||
		lp.Plane0 != (sectionrecord.PlaneRecord{Origin: lp.Frame0.Origin(), U: lp.Frame0.U(), V: lp.Frame0.V()}) ||
		lp.Plane1 != (sectionrecord.PlaneRecord{Origin: lp.Frame1.Origin(), U: lp.Frame1.U(), V: lp.Frame1.V()}) ||
		lp.Frame0.U() != lp.Frame1.U() || lp.Frame0.V() != lp.Frame1.V() || len(lp.Alignment) > 1 {
		return false, nil
	}
	for _, offset := range lp.Alignment {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if offset != 0 {
			return false, nil
		}
	}
	for _, level := range source.capLevels {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !loftVolumeSourceRatOK(level) {
			return false, nil
		}
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pp.Frame, Xform: pp.Xform},
		prismplacement.Operand{Frame: lp.Frame1, Xform: lp.Xform})
	prismTop := proofarith.FloatRat(pp.Z1)
	if !axis.OK || axis.Shift == nil || prismTop == nil ||
		!loftVolumeSourceRatOK(axis.Shift) || !loftVolumeSourceRatOK(prismTop) ||
		axis.Shift.Sign() <= 0 {
		return false, nil
	}
	comparison, bounded := loftVolumeSourceCmp(axis.Shift, prismTop)
	if !bounded || comparison >= 0 || source.capLevels[0].Sign() != 0 {
		return false, nil
	}
	comparison, bounded = loftVolumeSourceCmp(source.capLevels[1], axis.Shift)
	if !bounded || comparison != 0 {
		return false, nil
	}
	comparison, bounded = loftVolumeSourceCmp(source.capLevels[2], prismTop)
	if !bounded || comparison != 0 {
		return false, nil
	}
	for _, vertex := range shell.vertices {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		for _, value := range []*big.Rat{vertex.point.u, vertex.point.v, vertex.point.z,
			vertex.world[0], vertex.world[1], vertex.world[2]} {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !loftVolumeSourceRatOK(value) {
				return false, nil
			}
		}
	}
	prismLines, ok, err := loftPartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return false, err
	}
	bottom, ok, err := loftPartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return false, err
	}
	top, ok, err := loftPartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(bottom) != len(top) || len(source.walls) != len(bottom) ||
		source.carrier < 0 || source.carrier >= len(prismLines) {
		return false, err
	}
	for _, frame := range []r3.Frame{pp.Frame, lp.Frame0, lp.Frame1} {
		for _, vector := range []r3.Vec{frame.Origin(), frame.U(), frame.V(), frame.N()} {
			for _, value := range []float64{vector.X, vector.Y, vector.Z} {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if !loftVolumeSourceRatOK(proofarith.FloatRat(value)) {
					return false, nil
				}
			}
		}
	}
	for _, lines := range [][]sectionrecord.LineSeg{prismLines, bottom, top} {
		for _, line := range lines {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			for _, value := range []float64{line.Start.U, line.Start.V, line.End.U, line.End.V} {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if !loftVolumeSourceRatOK(proofarith.FloatRat(value)) {
					return false, nil
				}
			}
		}
	}
	comparison, bounded = loftVolumeSourceCmp(source.capLevels[0], source.capLevels[1])
	if !bounded || comparison >= 0 {
		return false, nil
	}
	comparison, bounded = loftVolumeSourceCmp(source.capLevels[1], source.capLevels[2])
	if !bounded || comparison >= 0 || source.topWallJoins != len(prismLines) {
		return false, nil
	}
	convex, err := loftPartitionConvexPrism(budget, prismLines)
	if err != nil || !convex {
		return false, err
	}
	// CertifyCarrierOccupancy proves strict convexity, exactly two transverse
	// crossings, and their order for the whole height interval. Agreement of
	// two seam rational functions at four distinct arguments is an identity:
	// after cross multiplication their difference has degree at most three.
	// A nonzero LineSeam is built only by CertifyLineSeam from finite float64
	// endpoints; its fields are private. Each input has a common dyadic
	// denominator 2^1074 and a numerator below 2099 bits. Coordinate
	// differences, crosses, and the one sum of crosses give coefficients below
	// 4203 numerator bits over 2^2148. At 0, 1/3, 2/3, and 1, polynomial
	// evaluation and quotient produce rationals below 6400 bits per part.
	// Thus At cannot exceed the 8192-bit ceiling before its result is checked.
	// A zero seam is refused by At; the four comparisons authenticate the
	// supplied seam against a freshly certified seam from these source lines.
	carrier := prismLines[source.carrier]
	crossing, seams, sides, admitted, err := loftcontact.CertifyCarrierOccupancy(ctx, budget,
		bottom, top, [2]sectionrecord.Point2{carrier.Start, carrier.End})
	if err != nil || !admitted || crossing != source.crossing {
		return false, err
	}
	for k := range seams {
		for _, t := range []*big.Rat{big.NewRat(0, 1), big.NewRat(1, 3),
			big.NewRat(2, 3), big.NewRat(1, 1)} {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b, validA := seams[k].At(t)
			c, d, validB := source.seams[k].At(t)
			if !validA || !validB || !loftVolumeSourceRatOK(a) || !loftVolumeSourceRatOK(b) ||
				!loftVolumeSourceRatOK(c) || !loftVolumeSourceRatOK(d) {
				return false, nil
			}
			comparison, bounded = loftVolumeSourceCmp(a, c)
			if !bounded || comparison != 0 {
				return false, nil
			}
			comparison, bounded = loftVolumeSourceCmp(b, d)
			if !bounded || comparison != 0 {
				return false, nil
			}
		}
	}
	// Every noncarrier prism halfplane has one strict, unchanged sign at
	// both Loft ends. The signed side of an interpolated vertex is affine in
	// height, so no other prism wall can acquire a hidden crossing.
	for i, line := range prismLines {
		if i == source.carrier {
			continue
		}
		for j := range bottom {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b := arithmetic.partitionSide(line, bottom[j].Start),
				arithmetic.partitionSide(line, top[j].Start)
			if !arithmetic.ok || a.Sign() <= 0 || b.Sign() <= 0 ||
				!loftVolumeSourceRatOK(a) || !loftVolumeSourceRatOK(b) {
				return false, nil
			}
		}
	}
	var expectedWalls []int8
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return false, err
		}
		left, right := sides[i], sides[(i+1)%len(sides)]
		crossed := i == crossing[0] || i == crossing[1]
		switch {
		case left > 0 && right > 0 && !crossed:
			expectedWalls = append(expectedWalls, -1)
		case left < 0 && right < 0 && !crossed:
			expectedWalls = append(expectedWalls, 1)
		case left != right && crossed:
			expectedWalls = append(expectedWalls, 0)
		default:
			return false, nil
		}
		if source.walls[i] != expectedWalls[i] {
			return false, nil
		}
	}
	if sides[crossing[0]] == sides[(crossing[0]+1)%len(sides)] ||
		sides[crossing[1]] == sides[(crossing[1]+1)%len(sides)] ||
		sides[crossing[0]] == sides[crossing[1]] {
		return false, nil
	}
	// The cap constructors recertify Sketch's selected cells and their exact
	// exposed boundary. The side constructor uses the same whole-height seam
	// identities just checked and refuses nonplanar wall patches. Rebuilding
	// in a separate shell avoids relying on labels supplied by the candidate.
	want := loftOutputNewShell()
	for capIndex := range 3 {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		valid, err := loftOutputCap(ctx, budget, pp, prismLines, bottom, top, source, &want, capIndex, arithmetic)
		if err != nil || !valid || !arithmetic.ok {
			return false, err
		}
	}
	valid, err := loftOutputSides(budget, pp, prismLines, bottom, top, source, &want, arithmetic)
	if err != nil || !valid || !arithmetic.ok {
		return false, err
	}
	valid, err = loftOutputAuditSource(budget, want, source, arithmetic)
	if err != nil || !valid || !arithmetic.ok {
		return false, err
	}
	valid, err = loftOutputAuditSource(budget, shell, source, arithmetic)
	if err != nil || !valid || !arithmetic.ok {
		return false, err
	}
	if len(want.vertices) != len(shell.vertices) || len(want.triangles) != len(shell.triangles) {
		return false, nil
	}
	for i, vertex := range want.vertices {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		other := shell.vertices[i]
		for _, pair := range [][2]*big.Rat{
			{vertex.point.u, other.point.u}, {vertex.point.v, other.point.v},
			{vertex.point.z, other.point.z}, {vertex.world[0], other.world[0]},
			{vertex.world[1], other.world[1]}, {vertex.world[2], other.world[2]},
		} {
			if err := budget.Step(); err != nil {
				return false, err
			}
			comparison, bounded = loftVolumeSourceCmp(pair[0], pair[1])
			if !bounded || comparison != 0 {
				return false, nil
			}
		}
	}
	for i := range want.triangles {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		if want.triangles[i] != shell.triangles[i] || want.faces[i] != shell.faces[i] {
			return false, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return arithmetic.ok, nil
}

func loftVolumeSourceRatOK(x *big.Rat) bool {
	return x != nil && x.Num().BitLen() <= loftVolumeSourceMaxRatBits &&
		x.Denom().BitLen() <= loftVolumeSourceMaxRatBits
}

// Rat.Cmp cross-multiplies both numerator/denominator pairs. Refuse before
// either raw product could exceed the private 8192-bit arithmetic ceiling.
func loftVolumeSourceCmp(a, b *big.Rat) (int, bool) {
	if !loftVolumeSourceRatOK(a) || !loftVolumeSourceRatOK(b) ||
		a.Num().BitLen()+b.Denom().BitLen() > loftVolumeSourceMaxRatBits ||
		b.Num().BitLen()+a.Denom().BitLen() > loftVolumeSourceMaxRatBits {
		return 0, false
	}
	return a.Cmp(b), true
}
