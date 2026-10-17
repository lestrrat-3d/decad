package prismpattern

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
)

// FilletProofBounds adds the complete local 3r disks to the joined prism's
// occupied-volume and area allowances.
func FilletProofBounds(meshBound, oldVolSymDiff, oldAreaSlack, arcBound,
	radius, height float64, contactCount int) (float64, float64, error) {
	piUpper := proofbound.RatFloatUp(proofbound.PiUpper)
	trueDisk := proofbound.ProductUpper(3, radius)
	heldDisk := proofbound.AbsSumUpper(trueDisk, meshBound, arcBound)
	if height <= 0 || proofbound.IsNonFinite(heldDisk) {
		return 0, 0, fmt.Errorf("%w: joined-prism fillet has no finite local disk", decaderr.ErrUnsupported)
	}
	n := float64(contactCount)
	trueDiskArea := proofbound.ProductUpper(piUpper, proofbound.ProductUpper(trueDisk, trueDisk))
	heldDiskArea := proofbound.ProductUpper(piUpper, proofbound.ProductUpper(heldDisk, heldDisk))
	volumeBand := proofbound.ProductUpper(n, proofbound.ProductUpper(height,
		proofbound.AbsSumUpper(trueDiskArea, heldDiskArea)))
	volumeGap := proofbound.AbsSumUpper(oldVolSymDiff, volumeBand)
	// Old and new walls inside each disk have total area below four full
	// cylinder walls; both changed caps fit in four complete disk faces.
	areaBand := proofbound.ProductUpper(n, proofbound.ProductUpper(4*piUpper,
		proofbound.AbsSumUpper(proofbound.ProductUpper(heldDisk, height),
			proofbound.ProductUpper(heldDisk, heldDisk))))
	areaSlack := proofbound.AbsSumUpper(oldAreaSlack, areaBand)
	if proofbound.IsNonFinite(volumeGap) || proofbound.IsNonFinite(areaSlack) {
		return 0, 0, fmt.Errorf("%w: joined-prism fillet has no finite volume or area proof",
			decaderr.ErrUnsupported)
	}
	return volumeGap, areaSlack, nil
}
