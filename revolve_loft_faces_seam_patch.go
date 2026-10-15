package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/r3"
)

// revolveLoftFacesSeamPatch is one open source-face patch split through a
// common held seam. Vertex bounds cover stations only; a complete face bound
// also needs the cell's curve and twist proofs before this enters a Boolean.
type revolveLoftFacesSeamPatch struct {
	vertices     []r3.Vec
	triangles    [][3]int
	vertexBounds []float64
	origins      []provenance.Ref
	seam         []int
	stationIDs   []int
}

// revolveLoftFacesArcPatchesUnchecked splits the two source parameter strips at the
// same certified radial-arc crossing. The blank strip is rebuilt from its
// exact straight meridian; the LoftFaces strip uses the original cap maps.
// Every middle-row coordinate is copied from the same seam vertex. These
// open patches are construction pieces, not a Boolean result or a contact
// admission; their outer edges still need the adjacent caps and side cells.
func revolveLoftFacesArcPatchesUnchecked(ctx context.Context, trim *loftFacesTrimRecord,
	band, segment int, section revolveSectionSpan, seam []revolveLoftFacesSeamVertex,
) (revolveLoftFacesSeamPatch, revolveLoftFacesSeamPatch, bool, error) {
	var blank, loft revolveLoftFacesSeamPatch
	if ctx == nil || trim == nil || trim.source == nil ||
		band < 0 || band+1 >= len(trim.maps) || segment < 0 ||
		len(seam) < 2 || len(trim.stationGroup) != len(trim.source.sourceStation) ||
		section.face == nil || len(section.origins) == 0 ||
		section.start.U == section.end.U ||
		section.start.V <= 0 || section.end.V <= 0 ||
		len(trim.source.profile.Outer.Segments) == 0 {
		return blank, loft, false, nil
	}
	count := len(trim.source.sourceStation)
	for i := range seam {
		if seam[i].station < 0 || seam[i].station >= count ||
			proofbound.IsNonFinite(seam[i].blankBound) ||
			proofbound.IsNonFinite(seam[i].loftBound) ||
			seam[i].blankBound < 0 || seam[i].loftBound < 0 {
			return blank, loft, false, nil
		}
		if i+1 < len(seam) &&
			((seam[i].station+1)%count != seam[i+1].station ||
				trim.stationGroup[seam[i].station] != segment) {
			return blank, loft, false, nil
		}
	}
	group := band*len(trim.source.profile.Outer.Segments) + segment
	if group < 0 || group >= len(trim.base.Groups) ||
		len(trim.base.Groups[group].Origins) == 0 {
		return blank, loft, false, nil
	}
	blank.origins = append([]provenance.Ref(nil), section.origins...)
	loft.origins = append([]provenance.Ref(nil), trim.base.Groups[group].Origins...)
	blank.vertices, loft.vertices = make([]r3.Vec, 3*len(seam)), make([]r3.Vec, 3*len(seam))
	blank.vertexBounds = make([]float64, 3*len(seam))
	loft.vertexBounds = make([]float64, 3*len(seam))
	blank.seam, loft.seam = make([]int, len(seam)), make([]int, len(seam))
	for j, entry := range seam {
		if err := ctx.Err(); err != nil {
			return blank, loft, false, err
		}
		station := trim.source.sourceStation[entry.station]
		u, v := lfF(station.U), lfF(station.V)
		for row, mapped := range [...]loftFaceMap{trim.maps[band], trim.maps[band+1]} {
			position := row * 2 * len(seam)
			if row == 1 {
				position = 2 * len(seam)
			}
			point, round, ok := revolveLoftFacesHeldPoint(mapped.At(u, v))
			if !ok {
				return blank, loft, false, nil
			}
			loft.vertices[position+j] = point
			loft.vertexBounds[position+j] = proofbound.AbsSumUpper(round,
				proofbound.ProductUpper(trim.source.sourceStationRound, mapped.Stretch))
		}
		for row, end := range [...]planePoint{section.start, section.end} {
			position := row * 2 * len(seam)
			if row == 1 {
				position = 2 * len(seam)
			}
			point, bound, ok := revolveLoftFacesMeridianPoint(entry.exact, end)
			if !ok {
				return blank, loft, false, nil
			}
			blank.vertices[position+j], blank.vertexBounds[position+j] = point, bound
		}
		middle := len(seam) + j
		blank.vertices[middle], loft.vertices[middle] = entry.point, entry.point
		blank.vertexBounds[middle], loft.vertexBounds[middle] = entry.blankBound, entry.loftBound
		blank.seam[j], loft.seam[j] = middle, middle
	}
	for j := 0; j+1 < len(seam); j++ {
		for row := range 2 {
			a, b := row*len(seam)+j, (row+1)*len(seam)+j
			triangles := [][3]int{{a, b, b + 1}, {a, b + 1, a + 1}}
			blank.triangles = append(blank.triangles, triangles...)
			loft.triangles = append(loft.triangles, triangles...)
		}
	}
	for _, bounds := range [][]float64{blank.vertexBounds, loft.vertexBounds} {
		for _, bound := range bounds {
			if proofbound.IsNonFinite(bound) || bound < 0 {
				return revolveLoftFacesSeamPatch{}, revolveLoftFacesSeamPatch{}, false, nil
			}
		}
	}
	return blank, loft, true, nil
}

// The endpoints remain on the same exact radial ray as the mapped station.
// This is a split of a source meridian cell, with radial and float movement
// separately charged; angular chording is not included in this vertex bound.
func revolveLoftFacesMeridianPoint(seam loftFaceRatVec, end planePoint) (r3.Vec, float64, bool) {
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
	// The stored square root chooses a radial scale. The exact residual of
	// that choice is charged below, so its rounding cannot admit contact.
	scale := lfDiv(lfF(end.V), lfF(seamRadius))
	exact := loftFaceRatVec{lfF(end.U), lfMul(seam[1], scale), lfMul(seam[2], scale)}
	point, round, ok := revolveLoftFacesHeldPoint(exact)
	if !ok {
		return r3.Vec{}, 0, false
	}
	radius := lfF(end.V)
	difference := lfAbs(lfSub(lfAdd(lfMul(exact[1], exact[1]),
		lfMul(exact[2], exact[2])), lfMul(radius, radius)))
	residual := proofbound.RatFloatUp(lfDiv(difference, radius))
	bound := proofbound.AbsSumUpper(residual, round)
	return point, bound, !proofbound.IsNonFinite(bound)
}
