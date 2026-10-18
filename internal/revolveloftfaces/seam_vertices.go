package revolveloftfaces

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ArcSeamVertex holds one shared source-mapped point and independent bounds
// for the finite Revolve side and the mapped LoftFaces side.
type ArcSeamVertex struct {
	Station    int
	Exact      loftface.RatVec
	Point      r3.Vec
	BlankBound float64
	LoftBound  float64
}

func seamZero() *big.Rat { return new(big.Rat) }
func seamOne() *big.Rat  { return big.NewRat(1, 1) }
func seamDiv(a, b *big.Rat) *big.Rat {
	return proofarith.QuoRat(new(big.Rat), a, b)
}
func seamAbs(a *big.Rat) *big.Rat { return new(big.Rat).Abs(a) }

// ArcSeamVerticesUnchecked places a held ring at one rational time inside a
// certified root bracket. The caller must authenticate that every station
// belongs, in order, to one whole radial Sketch Arc and that the bracket
// encloses its unique finite crossing for the complete arc. Held facets and
// station coincidence do not certify source contact.
func ArcSeamVerticesUnchecked(ctx context.Context, trim *loftFacesTrimRecord,
	band int, stations []int, root loftFaceInterval, section revolveSectionSpan,
) ([]ArcSeamVertex, bool, error) {
	if ctx == nil || trim == nil || trim.Source == nil ||
		band < 0 || band+1 >= len(trim.Maps) || len(stations) < 2 ||
		root.Lo == nil || root.Hi == nil || section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 ||
		root.Lo.Sign() < 0 || root.Hi.Cmp(seamOne()) > 0 || root.Lo.Cmp(root.Hi) > 0 ||
		proofbound.IsNonFinite(trim.Source.SourceStationRound) ||
		trim.Source.SourceStationRound < 0 {
		return nil, false, nil
	}
	first, last := trim.Maps[band], trim.Maps[band+1]
	xStart, xEnd := lfF(section.Start.U), lfF(section.End.U)
	rStart, rEnd := lfF(section.Start.V), lfF(section.End.V)
	meridianSlope := seamDiv(lfSub(rEnd, rStart), lfSub(xEnd, xStart))
	meridianOffset := lfSub(rStart, lfMul(meridianSlope, xStart))
	xLo, xHi := xStart, xEnd
	if xLo.Cmp(xHi) > 0 {
		xLo, xHi = xHi, xLo
	}
	stretch := max(first.Stretch, last.Stretch)
	if proofbound.IsNonFinite(stretch) || stretch < 0 {
		return nil, false, nil
	}
	stationBound := proofbound.ProductUpper(trim.Source.SourceStationRound, stretch)
	if proofbound.IsNonFinite(stationBound) {
		return nil, false, nil
	}
	mid := seamDiv(lfAdd(root.Lo, root.Hi), big.NewRat(2, 1))
	halfWidth := lfSub(mid, root.Lo)
	seen := make(map[int]struct{}, len(stations))
	vertices := make([]ArcSeamVertex, 0, len(stations))
	for _, stationIndex := range stations {
		if err := ctx.Err(); err != nil {
			return nil, false, err
		}
		if stationIndex < 0 || stationIndex >= len(trim.Source.SourceStation) {
			return nil, false, nil
		}
		if _, exists := seen[stationIndex]; exists {
			return nil, false, nil
		}
		seen[stationIndex] = struct{}{}
		station := trim.Source.SourceStation[stationIndex]
		u, v := lfF(station.U), lfF(station.V)
		a, z := first.At(u, v), last.At(u, v)
		for _, time := range []*big.Rat{root.Lo, root.Hi} {
			x := lfAdd(a[0], lfMul(time, lfSub(z[0], a[0])))
			if x.Cmp(xLo) <= 0 || x.Cmp(xHi) >= 0 {
				return nil, false, nil
			}
		}
		var exact loftface.RatVec
		timeDelta := seamZero()
		for axis := range exact {
			difference := lfSub(z[axis], a[axis])
			exact[axis] = lfAdd(a[axis], lfMul(mid, difference))
			timeDelta = lfAdd(timeDelta, lfMul(halfWidth, seamAbs(difference)))
		}
		held, roundBound, ok := HeldPoint(exact)
		if !ok {
			return nil, false, nil
		}
		radius := lfAdd(lfMul(meridianSlope, exact[0]), meridianOffset)
		if radius.Sign() <= 0 {
			return nil, false, nil
		}
		// |sqrt(Y²+Z²)-R| = |Y²+Z²-R²|/(sqrt(Y²+Z²)+R).
		// The positive R alone is a safe denominator for this bound.
		radialSquare := lfAdd(lfMul(exact[1], exact[1]), lfMul(exact[2], exact[2]))
		radialGap := seamDiv(seamAbs(lfSub(radialSquare, lfMul(radius, radius))), radius)
		blankBound := proofbound.AbsSumUpper(proofbound.RatFloatUp(radialGap), roundBound)
		loftBound := proofbound.AbsSumUpper(proofbound.RatFloatUp(timeDelta), stationBound, roundBound)
		if proofbound.IsNonFinite(blankBound) || proofbound.IsNonFinite(loftBound) {
			return nil, false, nil
		}
		vertices = append(vertices, ArcSeamVertex{
			Station: stationIndex, Exact: exact, Point: held,
			BlankBound: blankBound, LoftBound: loftBound})
	}
	return vertices, true, nil
}

// HeldPoint rounds an exact source-mapped point once and charges its entire
// Euclidean movement by the rational L1 upper bound.
func HeldPoint(exact loftface.RatVec) (r3.Vec, float64, bool) {
	var held r3.Vec
	roundDelta := seamZero()
	for axis, coordinate := range exact {
		if coordinate == nil {
			return r3.Vec{}, 0, false
		}
		value, _ := coordinate.Float64()
		if proofbound.IsNonFinite(value) {
			return r3.Vec{}, 0, false
		}
		roundDelta = lfAdd(roundDelta, seamAbs(lfSub(lfF(value), coordinate)))
		switch axis {
		case 0:
			held.X = value
		case 1:
			held.Y = value
		case 2:
			held.Z = value
		}
	}
	bound := proofbound.RatFloatUp(roundDelta)
	return held, bound, !proofbound.IsNonFinite(bound)
}
