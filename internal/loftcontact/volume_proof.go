package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismplacement"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// VolumeComposerRatBits is the size ceiling VolumeRatFits applies to the
// numerator and denominator of every rational VolumeCompose reads or keeps.
const VolumeComposerRatBits = 8192

// VolumeOutputSteps is the output-phase work limit shared by the boundary
// proof, VolumeCompose and the shell published from them.
const VolumeOutputSteps = 65_536

// VolumeProof is transient. Its bound is meaningful only for the source
// partition and boundary proof certified from the same live operands.
// SymDiffUpper bounds the symmetric difference between the exact source
// shell and the held shell; TotalOutputVisits counts the boundary and
// composer steps together.
type VolumeProof struct {
	Boundary          OutputBoundary
	SourceVolume      *big.Rat
	SymDiffUpper      float64
	TotalOutputVisits uint64
}

// VolumeCompose uses the source partition carried by its boundary proof.
// The payloads must come from the same live records passed to that proof.
func VolumeCompose(ctx context.Context, operation *proofbound.WorkBudget,
	pp prismrecord.Payload, lp loftrecord.Payload, boundary OutputBoundary,
) (VolumeProof, bool, error) {
	var no VolumeProof
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if operation == nil || boundary.WorkVisits == 0 ||
		boundary.WorkVisits >= VolumeOutputSteps ||
		len(boundary.Shell.Triangles) == 0 || len(boundary.Shell.Triangles) > 512 ||
		len(boundary.Shell.Vertices) == 0 || len(boundary.Shell.Vertices) > 512 {
		return no, false, nil
	}
	limit := proofbound.NewWallWorkBudgetWithOperation(
		VolumeOutputSteps-boundary.WorkVisits, operation)
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
	shell := boundary.Shell
	if admitted, err := VolumeSourceEquality(ctx, budget, pp, lp, boundary.Source, shell); err != nil || !admitted {
		return no, false, err
	}
	world := make([][3]*big.Rat, len(shell.Vertices))
	for i, vertex := range shell.Vertices {
		world[i] = vertex.World
	}
	sourceVolume, admitted, err := VolumeExactShell(budget, world, shell.Triangles)
	if err != nil || !admitted {
		return no, false, err
	}
	if !VolumeRatFits(sourceVolume) || sourceVolume.Sign() <= 0 {
		return no, false, nil
	}
	bound := new(big.Rat)
	for _, tri := range shell.Triangles {
		if err := ctx.Err(); err != nil {
			return no, false, err
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		var points [6][3]*big.Rat
		for corner, index := range tri {
			if index < 0 || index >= len(shell.Vertices) {
				return no, false, nil
			}
			vertex := shell.Vertices[index]
			for axis, coordinate := range vertex.World {
				if !VolumeRatFits(coordinate) {
					return no, false, nil
				}
				points[corner][axis] = coordinate
			}
			for axis, coordinate := range [3]float64{vertex.Held.X, vertex.Held.Y, vertex.Held.Z} {
				if proofbound.IsNonFinite(coordinate) {
					return no, false, nil
				}
				if err := budget.Step(); err != nil {
					return no, false, err
				}
				points[corner+3][axis] = proofarith.FloatRat(coordinate)
				if !VolumeRatFits(points[corner+3][axis]) {
					return no, false, nil
				}
			}
		}
		term, admitted, err := VolumeHullBound(budget, points)
		if err != nil || !admitted {
			return no, false, err
		}
		if !VolumeRatFits(term) || term.Sign() < 0 || !VolumeSumFits(bound, term) {
			return no, false, nil
		}
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		bound.Add(bound, term)
		if !VolumeRatFits(bound) {
			return no, false, nil
		}
	}
	if err := budget.Step(); err != nil {
		return no, false, err
	}
	upper, rounded := VolumeRoundUp(bound)
	if !rounded {
		return no, false, nil
	}
	if proofbound.IsNonFinite(upper) || upper < 0 {
		return no, false, nil
	}
	upperRat := proofarith.FloatRat(upper)
	if !VolumeCompareFits(sourceVolume, upperRat) || sourceVolume.Cmp(upperRat) <= 0 {
		return no, false, nil
	}
	if err := ctx.Err(); err != nil {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	return VolumeProof{
		Boundary: boundary, SourceVolume: new(big.Rat).Set(sourceVolume),
		SymDiffUpper: upper, TotalOutputVisits: boundary.WorkVisits + volumeVisits,
	}, true, nil
}

// VolumeRatFits reports whether value is non-nil and its numerator and
// denominator are within VolumeComposerRatBits.
func VolumeRatFits(value *big.Rat) bool {
	return value != nil && value.Denom().Sign() > 0 &&
		value.Num().BitLen() <= VolumeComposerRatBits &&
		value.Denom().BitLen() <= VolumeComposerRatBits
}

// VolumeRoundUp preflights the integer alignment used by RatFloat64 and
// the cross products used by the exact rounding-direction comparison.
func VolumeRoundUp(value *big.Rat) (float64, bool) {
	if !VolumeRatFits(value) || value.Sign() < 0 {
		return 0, false
	}
	if value.Sign() == 0 {
		return 0, true
	}
	numeratorBits, denominatorBits := value.Num().BitLen(), value.Denom().BitLen()
	if value.Denom().TrailingZeroBits() != uint(denominatorBits-1) {
		shift := 54 - (numeratorBits - denominatorBits)
		if shift > 0 && numeratorBits+shift > VolumeComposerRatBits {
			return 0, false
		}
	}
	nearest, _ := proofarith.RatFloat64(value)
	if proofbound.IsNonFinite(nearest) {
		return 0, false
	}
	nearestRat := proofarith.FloatRat(nearest)
	if !VolumeCompareFits(nearestRat, value) {
		return 0, false
	}
	if nearestRat.Cmp(value) < 0 {
		nearest = proofarith.NextUp(nearest)
	}
	return nearest, !proofbound.IsNonFinite(nearest)
}

// Adding a/b and c/d can need ad+bc over bd before reduction. Refuse before
// that allocation if either unreduced intermediate could pass the ceiling.
func VolumeSumFits(a, b *big.Rat) bool {
	if !VolumeRatFits(a) || !VolumeRatFits(b) {
		return false
	}
	if a.Sign() == 0 || b.Sign() == 0 {
		return true
	}
	an, ad := a.Num().BitLen(), a.Denom().BitLen()
	bn, bd := b.Num().BitLen(), b.Denom().BitLen()
	return max(an+bd, bn+ad)+1 <= VolumeComposerRatBits &&
		ad+bd <= VolumeComposerRatBits
}

// VolumeCompareFits reports whether comparing a and b keeps both cross
// products within VolumeComposerRatBits.
func VolumeCompareFits(a, b *big.Rat) bool {
	return VolumeRatFits(a) && VolumeRatFits(b) &&
		a.Num().BitLen()+b.Denom().BitLen() <= VolumeComposerRatBits &&
		b.Num().BitLen()+a.Denom().BitLen() <= VolumeComposerRatBits
}

// VolumeSourceEquality proves that the exact triangles in shell tile the
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
func VolumeSourceEquality(ctx context.Context, budget *proofbound.WorkBudget,
	pp prismrecord.Payload, lp loftrecord.Payload, source SourcePartition, shell OutputShell) (bool, error) {
	arithmetic := &Arithmetic{OK: true}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if budget == nil || len(shell.Vertices) == 0 || len(shell.Vertices) > 512 ||
		len(shell.Triangles) == 0 || len(shell.Triangles) > 512 ||
		len(shell.Faces) != len(shell.Triangles) || len(shell.TriangleBounds) != len(shell.Triangles) ||
		len(source.Walls) < 3 || len(source.Walls) > 32 ||
		source.CapLevels[0] == nil || source.CapLevels[1] == nil || source.CapLevels[2] == nil {
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
	if !loftPrismSourcePairEligible(pp, lp) {
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
	for _, level := range source.CapLevels {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !arithmetic.Valid(level) {
			return false, nil
		}
	}
	axis := prismplacement.SharedAxisOf(
		prismplacement.Operand{Frame: pp.Frame, Xform: pp.Xform},
		prismplacement.Operand{Frame: lp.Frame1, Xform: lp.Xform})
	prismTop := proofarith.FloatRat(pp.Z1)
	if !axis.OK || !arithmetic.Valid(axis.Shift) || !arithmetic.Valid(prismTop) ||
		axis.Shift.Sign() <= 0 {
		return false, nil
	}
	// A refused Valid or Cmp clears arithmetic.OK for good, and every check
	// below returns false once OK is clear.
	if arithmetic.Cmp(axis.Shift, prismTop) >= 0 || !arithmetic.OK || source.CapLevels[0].Sign() != 0 {
		return false, nil
	}
	if arithmetic.Cmp(source.CapLevels[1], axis.Shift) != 0 || !arithmetic.OK {
		return false, nil
	}
	if arithmetic.Cmp(source.CapLevels[2], prismTop) != 0 || !arithmetic.OK {
		return false, nil
	}
	for _, vertex := range shell.Vertices {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		for _, value := range []*big.Rat{vertex.Point.U, vertex.Point.V, vertex.Point.Z,
			vertex.World[0], vertex.World[1], vertex.World[2]} {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if !arithmetic.Valid(value) {
				return false, nil
			}
		}
	}
	prismLines, ok, err := PartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return false, err
	}
	bottom, ok, err := PartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return false, err
	}
	top, ok, err := PartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(bottom) != len(top) || len(source.Walls) != len(bottom) ||
		source.Carrier < 0 || source.Carrier >= len(prismLines) {
		return false, err
	}
	for _, frame := range []r3.Frame{pp.Frame, lp.Frame0, lp.Frame1} {
		for _, vector := range []r3.Vec{frame.Origin(), frame.U(), frame.V(), frame.N()} {
			for _, value := range []float64{vector.X, vector.Y, vector.Z} {
				if err := budget.Step(); err != nil {
					return false, err
				}
				if !arithmetic.Valid(proofarith.FloatRat(value)) {
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
				if !arithmetic.Valid(proofarith.FloatRat(value)) {
					return false, nil
				}
			}
		}
	}
	if arithmetic.Cmp(source.CapLevels[0], source.CapLevels[1]) >= 0 || !arithmetic.OK {
		return false, nil
	}
	if arithmetic.Cmp(source.CapLevels[1], source.CapLevels[2]) >= 0 || !arithmetic.OK ||
		source.TopWallJoins != len(prismLines) {
		return false, nil
	}
	convex, err := convexPrism(budget, prismLines)
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
	carrier := prismLines[source.Carrier]
	crossing, seams, sides, admitted, err := CertifyCarrierOccupancy(ctx, budget,
		bottom, top, [2]sectionrecord.Point2{carrier.Start, carrier.End})
	if err != nil || !admitted || crossing != source.Crossing {
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
			c, d, validB := source.Seams[k].At(t)
			if !validA || !validB || !arithmetic.Valid(a) || !arithmetic.Valid(b) ||
				!arithmetic.Valid(c) || !arithmetic.Valid(d) {
				return false, nil
			}
			if arithmetic.Cmp(a, c) != 0 || arithmetic.Cmp(b, d) != 0 || !arithmetic.OK {
				return false, nil
			}
		}
	}
	// Every noncarrier prism halfplane has one strict, unchanged sign at
	// both Loft ends. The signed side of an interpolated vertex is affine in
	// height, so no other prism wall can acquire a hidden crossing.
	for i, line := range prismLines {
		if i == source.Carrier {
			continue
		}
		for j := range bottom {
			if err := ctx.Err(); err != nil {
				return false, err
			}
			if err := budget.Step(); err != nil {
				return false, err
			}
			a, b := arithmetic.LineSide(line, bottom[j].Start),
				arithmetic.LineSide(line, top[j].Start)
			if !arithmetic.OK || a.Sign() <= 0 || b.Sign() <= 0 ||
				!arithmetic.Valid(a) || !arithmetic.Valid(b) {
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
		if source.Walls[i] != expectedWalls[i] {
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
	want := NewOutputShell()
	for capIndex := range 3 {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		valid, err := OutputCap(ctx, budget, pp.Frame, prismLines, bottom, top, source, &want, capIndex, arithmetic)
		if err != nil || !valid || !arithmetic.OK {
			return false, err
		}
	}
	valid, err := OutputSides(budget, pp.Frame, prismLines, bottom, top, source, &want, arithmetic)
	if err != nil || !valid || !arithmetic.OK {
		return false, err
	}
	valid, err = AuditOutputSource(budget, want, source, arithmetic)
	if err != nil || !valid || !arithmetic.OK {
		return false, err
	}
	valid, err = AuditOutputSource(budget, shell, source, arithmetic)
	if err != nil || !valid || !arithmetic.OK {
		return false, err
	}
	if len(want.Vertices) != len(shell.Vertices) || len(want.Triangles) != len(shell.Triangles) {
		return false, nil
	}
	for i, vertex := range want.Vertices {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		other := shell.Vertices[i]
		for _, pair := range [][2]*big.Rat{
			{vertex.Point.U, other.Point.U}, {vertex.Point.V, other.Point.V},
			{vertex.Point.Z, other.Point.Z}, {vertex.World[0], other.World[0]},
			{vertex.World[1], other.World[1]}, {vertex.World[2], other.World[2]},
		} {
			if err := budget.Step(); err != nil {
				return false, err
			}
			if arithmetic.Cmp(pair[0], pair[1]) != 0 || !arithmetic.OK {
				return false, nil
			}
		}
	}
	for i := range want.Triangles {
		if err := ctx.Err(); err != nil {
			return false, err
		}
		if err := budget.Step(); err != nil {
			return false, err
		}
		if want.Triangles[i] != shell.Triangles[i] || want.Faces[i] != shell.Faces[i] {
			return false, nil
		}
	}
	if err := ctx.Err(); err != nil {
		return false, err
	}
	if err := budget.Err(); err != nil {
		return false, err
	}
	return arithmetic.OK, nil
}
