package tessellation

import "github.com/lestrrat-3d/decad/internal/sectionrecord"

// CompactCapBlendRing omits zero-length contour edges where a fillet's
// inward circular offset has collapsed to its centre. The band's samples
// retain their full count so its strips still match the side wall.
func CompactCapBlendRing(pts []sectionrecord.Point2, vertices []int) ([]sectionrecord.Point2, []int) {
	compactPts := make([]sectionrecord.Point2, 0, len(pts))
	compactVertices := make([]int, 0, len(vertices))
	for i, p := range pts {
		if len(compactPts) != 0 && compactPts[len(compactPts)-1] == p {
			continue
		}
		compactPts = append(compactPts, p)
		compactVertices = append(compactVertices, vertices[i])
	}
	if len(compactPts) > 1 && compactPts[0] == compactPts[len(compactPts)-1] {
		compactPts = compactPts[:len(compactPts)-1]
		compactVertices = compactVertices[:len(compactVertices)-1]
	}
	return compactPts, compactVertices
}
