package revolveloftfaces

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftface"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// TangentToeStationLink ties the held vertices of the toe-cut tooth to the
// stations of the LoftFaces end ring: a vertex equal, as floats, to station
// vertex j of the last cap has station j. Chord j's bound is the certified
// distance between the held chord and the true cap-boundary piece
// (record.Bounds[last]). toe must be non-nil.
func TangentToeStationLink(vertices []r3.Vec, toe *facetedrecord.LoftFacesToeRecord) (loftface.StationLink, error) {
	refuse := func(why string) (loftface.StationLink, error) {
		return loftface.StationLink{}, fmt.Errorf("%w: the tangent tooth %s", decaderr.ErrUnsupported, why)
	}
	record := toe.Cap
	last := len(record.Maps) - 1
	n := len(record.Source.SourceStation)
	baseVerts := toe.Base.Verts
	if last < 1 || len(record.Bounds) != len(record.Maps) || len(baseVerts) < (last+1)*n || n < 3 {
		return refuse("has no station ring")
	}
	capBounds := record.Bounds[last]
	if len(capBounds.SourceDeparture) != n || len(capBounds.ProjectiveDeparture) != n {
		return refuse("has no chord bounds")
	}
	stationOf := make(map[r3.Vec]int, n)
	link := loftface.StationLink{Station: make([]int, len(vertices)), ChordBound: make([]float64, n)}
	for j := range n {
		v := baseVerts[last*n+j]
		if _, dup := stationOf[v]; dup {
			return refuse("has a repeated station vertex")
		}
		stationOf[v] = j
		link.ChordBound[j] = proofbound.AbsSumUpper(capBounds.SourceDeparture[j],
			capBounds.ProjectiveDeparture[j], capBounds.StationRound)
	}
	for i, v := range vertices {
		j, ok := stationOf[v]
		if !ok {
			j = -1
		}
		link.Station[i] = j
	}
	return link, nil
}
