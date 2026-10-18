package revolveloftfaces

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// ArcSeamPatch is an open source-face construction piece. Its vertex bounds
// exclude the complete face's curve and twist proofs.
type ArcSeamPatch struct {
	Vertices     []r3.Vec
	Triangles    [][3]int
	VertexBounds []float64
	Origins      []provenance.Ref
	Seam         []int
}

// ArcPatchesUnchecked splits two source parameter strips at the same
// certified radial-arc crossing. The blank strip is rebuilt from its exact
// straight meridian; the LoftFaces strip uses the original cap maps. Every
// middle-row coordinate is copied from the same seam vertex. These open
// patches are construction pieces, not a Boolean result or contact admission.
// The caller must first authenticate the live source pair and station run.
func ArcPatchesUnchecked(ctx context.Context, trim *loftFacesTrimRecord,
	band, segment int, section revolveSectionSpan, seam []ArcSeamVertex,
) (ArcSeamPatch, ArcSeamPatch, bool, error) {
	var blank, loft ArcSeamPatch
	if ctx == nil || trim == nil || trim.Source == nil ||
		band < 0 || band+1 >= len(trim.Maps) || segment < 0 ||
		len(seam) < 2 || len(trim.StationGroup) != len(trim.Source.SourceStation) ||
		section.Face == nil || len(section.Origins) == 0 ||
		section.Start.U == section.End.U ||
		section.Start.V <= 0 || section.End.V <= 0 ||
		len(trim.Source.Profile.Outer.Segments) == 0 {
		return blank, loft, false, nil
	}
	count := len(trim.Source.SourceStation)
	for i := range seam {
		if seam[i].Station < 0 || seam[i].Station >= count ||
			proofbound.IsNonFinite(seam[i].BlankBound) ||
			proofbound.IsNonFinite(seam[i].LoftBound) ||
			seam[i].BlankBound < 0 || seam[i].LoftBound < 0 {
			return blank, loft, false, nil
		}
		if i+1 < len(seam) &&
			((seam[i].Station+1)%count != seam[i+1].Station ||
				trim.StationGroup[seam[i].Station] != segment) {
			return blank, loft, false, nil
		}
	}
	group := band*len(trim.Source.Profile.Outer.Segments) + segment
	if group < 0 || group >= len(trim.Base.Groups) ||
		len(trim.Base.Groups[group].Origins) == 0 {
		return blank, loft, false, nil
	}
	blank.Origins = append([]provenance.Ref(nil), section.Origins...)
	loft.Origins = append([]provenance.Ref(nil), trim.Base.Groups[group].Origins...)
	blank.Vertices, loft.Vertices = make([]r3.Vec, 3*len(seam)), make([]r3.Vec, 3*len(seam))
	blank.VertexBounds = make([]float64, 3*len(seam))
	loft.VertexBounds = make([]float64, 3*len(seam))
	blank.Seam, loft.Seam = make([]int, len(seam)), make([]int, len(seam))
	for j, entry := range seam {
		if err := ctx.Err(); err != nil {
			return blank, loft, false, err
		}
		station := trim.Source.SourceStation[entry.Station]
		u, v := lfF(station.U), lfF(station.V)
		for row, mapped := range [...]loftFaceMap{trim.Maps[band], trim.Maps[band+1]} {
			position := row * 2 * len(seam)
			if row == 1 {
				position = 2 * len(seam)
			}
			point, round, ok := HeldPoint(mapped.At(u, v))
			if !ok {
				return blank, loft, false, nil
			}
			loft.Vertices[position+j] = point
			loft.VertexBounds[position+j] = proofbound.AbsSumUpper(round,
				proofbound.ProductUpper(trim.Source.SourceStationRound, mapped.Stretch))
		}
		for row, end := range [...]sectionrecord.Point2{section.Start, section.End} {
			position := row * 2 * len(seam)
			if row == 1 {
				position = 2 * len(seam)
			}
			point, bound, ok := meridianPoint(entry.Exact, end)
			if !ok {
				return blank, loft, false, nil
			}
			blank.Vertices[position+j], blank.VertexBounds[position+j] = point, bound
		}
		middle := len(seam) + j
		blank.Vertices[middle], loft.Vertices[middle] = entry.Point, entry.Point
		blank.VertexBounds[middle], loft.VertexBounds[middle] = entry.BlankBound, entry.LoftBound
		blank.Seam[j], loft.Seam[j] = middle, middle
	}
	for j := 0; j+1 < len(seam); j++ {
		for row := range 2 {
			a, b := row*len(seam)+j, (row+1)*len(seam)+j
			triangles := [][3]int{{a, b, b + 1}, {a, b + 1, a + 1}}
			blank.Triangles = append(blank.Triangles, triangles...)
			loft.Triangles = append(loft.Triangles, triangles...)
		}
	}
	for _, bounds := range [][]float64{blank.VertexBounds, loft.VertexBounds} {
		for _, bound := range bounds {
			if proofbound.IsNonFinite(bound) || bound < 0 {
				return ArcSeamPatch{}, ArcSeamPatch{}, false, nil
			}
		}
	}
	return blank, loft, true, nil
}

// meridianPoint keeps the endpoint on the exact radial ray of the seam.
// The radial residual and float movement are charged separately; angular
// chording is excluded from this vertex bound.
func meridianPoint(seam loftface.RatVec, end sectionrecord.Point2) (r3.Vec, float64, bool) {
	if seam[0] == nil || seam[1] == nil || seam[2] == nil || end.V <= 0 {
		return r3.Vec{}, 0, false
	}
	radialSquared := lfAdd(lfMul(seam[1], seam[1]), lfMul(seam[2], seam[2]))
	if radialSquared.Sign() <= 0 {
		return r3.Vec{}, 0, false
	}
	squared, _ := radialSquared.Float64()
	if proofbound.IsNonFinite(squared) || squared <= 0 {
		return r3.Vec{}, 0, false
	}
	seamRadius := math.Sqrt(squared)
	if proofbound.IsNonFinite(seamRadius) || seamRadius <= 0 {
		return r3.Vec{}, 0, false
	}
	// The stored square root chooses a radial scale. Its exact residual is
	// charged below, so rounding cannot admit contact.
	scale := seamDiv(lfF(end.V), lfF(seamRadius))
	exact := loftface.RatVec{lfF(end.U), lfMul(seam[1], scale), lfMul(seam[2], scale)}
	point, round, ok := HeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	radius := lfF(end.V)
	difference := seamAbs(lfSub(lfAdd(lfMul(exact[1], exact[1]),
		lfMul(exact[2], exact[2])), lfMul(radius, radius)))
	residual := proofbound.RatFloatUp(seamDiv(difference, radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}
