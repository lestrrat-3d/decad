package revolveshell

import (
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

type SectorBoreCaps struct {
	Low, High             int
	LowAxis, HighAxis     int
	LowOuter, HighOuter   []int
	LowGroup, HighGroup   int
	LowX, HighX           float64
	LowRadius, HighRadius float64
}

// RevolveSectorBoreCapRecord derives the cap faces and ring IDs from the retained
// meridian walk. The two neighbors of its sole axial span must be complete
// planar radial caps, regardless of the number of intervening section spans.
func RevolveSectorBoreCapRecord(joined facetedrecord.Payload) (SectorBoreCaps, bool) {
	var caps SectorBoreCaps
	record := joined.RevolveLoftJoin
	if record == nil || record.Certificate == nil || record.Blank == nil ||
		record.CapRing == nil || record.Certificate.Section.Blank != record.Blank ||
		record.BoundaryBound != joined.MeshBound ||
		record.VolumeBound != joined.VolSymDiff || record.AreaBound != joined.AreaSlack ||
		joined.Xform != r3.Identity() || !joined.ContactAudited {
		return caps, false
	}
	section, ring := record.Certificate.Section, record.CapRing
	spans := section.Spans
	count, n := len(spans), len(ring.Angles)
	if count < 4 || len(ring.Rings) != count || len(ring.Groups) != count ||
		section.AxisSpan < 0 || section.AxisSpan >= count ||
		n < 16 || n > revolvemesh.SectorBoreMaxStations ||
		len(joined.Tris) != len(joined.Src) ||
		len(joined.Verts) != len(joined.VertexBound) {
		return caps, false
	}
	axis := section.AxisSpan
	caps.Low, caps.High = (axis+count-1)%count, (axis+1)%count
	low, high := spans[caps.Low], spans[caps.High]
	if spans[axis].Start.V != 0 || spans[axis].End.V != 0 ||
		low.End != spans[axis].Start || high.Start != spans[axis].End ||
		low.Start.V <= 0 || high.End.V <= 0 ||
		low.Start.U != low.End.U || high.Start.U != high.End.U ||
		low.Start.U >= high.Start.U ||
		low.Face == nil || high.Face == nil ||
		len(low.Origins) == 0 || len(high.Origins) == 0 {
		return caps, false
	}
	if _, ok := low.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, false
	}
	if _, ok := high.Face.Surface.(surfacegeom.Plane); !ok {
		return caps, false
	}
	caps.LowX, caps.HighX = low.Start.U, high.Start.U
	caps.LowRadius, caps.HighRadius = low.Start.V, high.End.V
	caps.LowGroup, caps.HighGroup = ring.Groups[caps.Low], ring.Groups[caps.High]
	if caps.LowGroup < 0 || caps.HighGroup < 0 ||
		caps.LowGroup >= len(joined.Groups) || caps.HighGroup >= len(joined.Groups) ||
		caps.LowGroup == caps.HighGroup ||
		len(ring.Rings[axis]) != 1 || len(ring.Rings[caps.High]) != 1 ||
		len(ring.Rings[caps.Low]) != n ||
		len(ring.Rings[(caps.High+1)%count]) != n {
		return caps, false
	}
	caps.LowAxis, caps.HighAxis = ring.Vertex(axis, 0), ring.Vertex(caps.High, 0)
	caps.LowOuter = ring.Rings[caps.Low]
	caps.HighOuter = ring.Rings[(caps.High+1)%count]
	if caps.LowAxis == caps.HighAxis ||
		caps.LowAxis < 0 || caps.HighAxis < 0 ||
		caps.LowAxis >= len(joined.Verts) || caps.HighAxis >= len(joined.Verts) ||
		joined.Verts[caps.LowAxis] != (r3.Vec{X: caps.LowX}) ||
		joined.Verts[caps.HighAxis] != (r3.Vec{X: caps.HighX}) {
		return caps, false
	}
	return caps, true
}
