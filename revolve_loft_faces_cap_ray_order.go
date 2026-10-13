package decad

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

type revolveLoftFacesRayBox struct {
	y, z loftFaceInterval
}

// Each cap endpoint's whole source box must stay strictly between its two
// neighboring rays. A neighboring seam ray also carries its source movement
// bound. This keeps the sorted held ray walk in the certified source order.
func revolveLoftFacesCapRayOrderBounded(rays []revolveLoftFacesRay,
	seam []revolveLoftFacesSeamVertex,
	capVertices []revolveLoftFacesCapSeamVertex) bool {
	if len(capVertices) == 0 {
		return true
	}
	if len(rays) < 4 {
		return false
	}
	boxes := make([]revolveLoftFacesRayBox, len(rays))
	capRay := make([]int, len(capVertices))
	for i := range capRay {
		capRay[i] = -1
	}
	for index, ray := range rays {
		if ray.y == nil || ray.z == nil || len(ray.capIDs) > 1 {
			return false
		}
		box := revolveLoftFacesRayBox{
			y: loftFaceInterval{Lo: ray.y, Hi: ray.y},
			z: loftFaceInterval{Lo: ray.z, Hi: ray.z},
		}
		if ray.seamIndex >= 0 {
			if ray.seamIndex >= len(seam) || len(ray.capIDs) != 0 {
				return false
			}
			point := seam[ray.seamIndex]
			if proofbound.IsNonFinite(point.loftBound) || point.loftBound < 0 {
				return false
			}
			bound := lfF(point.loftBound)
			y, z := lfF(point.point.Y), lfF(point.point.Z)
			box.y = loftFaceInterval{Lo: lfSub(y, bound), Hi: lfAdd(y, bound)}
			box.z = loftFaceInterval{Lo: lfSub(z, bound), Hi: lfAdd(z, bound)}
		}
		if len(ray.capIDs) == 1 {
			id := ray.capIDs[0]
			if id < 0 || id >= len(capVertices) || capRay[id] >= 0 || ray.seamIndex >= 0 {
				return false
			}
			capRay[id] = index
			boundary := capVertices[id].boundary
			box.y, box.z = boundary.box[1], boundary.box[2]
		}
		if !revolveLoftFacesRayBoxValid(box) {
			return false
		}
		boxes[index] = box
	}
	for _, index := range capRay {
		if index < 0 {
			return false
		}
		prev := (index - 1 + len(rays)) % len(rays)
		next := (index + 1) % len(rays)
		if prev < index && !revolveLoftFacesRayBefore(rays[prev], rays[index]) ||
			next > index && !revolveLoftFacesRayBefore(rays[index], rays[next]) ||
			revolveLoftFacesRayCrossRange(boxes[prev], boxes[index]).Lo.Sign() <= 0 ||
			revolveLoftFacesRayCrossRange(boxes[index], boxes[next]).Lo.Sign() <= 0 {
			return false
		}
	}
	return true
}

func revolveLoftFacesRayBoxValid(box revolveLoftFacesRayBox) bool {
	return box.y.Lo != nil && box.y.Hi != nil && box.z.Lo != nil && box.z.Hi != nil &&
		box.y.Lo.Cmp(box.y.Hi) <= 0 && box.z.Lo.Cmp(box.z.Hi) <= 0
}

func revolveLoftFacesRayCrossRange(a, b revolveLoftFacesRayBox) loftFaceInterval {
	return loftface.IntervalSub(revolveLoftFacesRayProductRange(a.y, b.z),
		revolveLoftFacesRayProductRange(a.z, b.y))
}

func revolveLoftFacesRayProductRange(a, b loftFaceInterval) loftFaceInterval {
	products := [4]*big.Rat{
		lfMul(a.Lo, b.Lo), lfMul(a.Lo, b.Hi),
		lfMul(a.Hi, b.Lo), lfMul(a.Hi, b.Hi),
	}
	out := loftFaceInterval{Lo: products[0], Hi: products[0]}
	for _, product := range products[1:] {
		if product.Cmp(out.Lo) < 0 {
			out.Lo = product
		}
		if product.Cmp(out.Hi) > 0 {
			out.Hi = product
		}
	}
	return out
}
