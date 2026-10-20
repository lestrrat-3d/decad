// Package sweeprun runs the pair sweeps: it picks the producer whose source
// geometry admits two bodies and their paths, and fills a sweep report with
// the certified event, bracket, track and replay proof. It works on topology
// nodes; the root package lifts the finished report to its handles.
package sweeprun

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/seal"
	"github.com/lestrrat-3d/decad/internal/sweeppath"
	"github.com/lestrrat-3d/decad/internal/sweepproof"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
)

// Hooks are the services a sweep takes from the document it runs in.
type Hooks struct {
	// Contact classifies the pair at two poses as ContactPair does, in node
	// form, and returns a copy the caller may keep.
	Contact func(ctx context.Context, poseA, poseB r3.Transform, req vocab.ContactRequest) (*contactReport, error)
	// Mesher meshes a held body for the faceted and planar paths.
	Mesher verifiedmesh.Mesher
}

// The sweep records of the public report package, with topology nodes as the
// body, face, edge and vertex references.
type (
	sweepReport = vocab.SweepReportOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex, *sweepproof.Proof]
	sweepSample = vocab.SweepSampleOf[*topo.Body, *topo.Face, *topo.Edge, *topo.Vertex]
	sweepEvent  = vocab.SweepEventOf[*topo.Face, *topo.Edge, *topo.Vertex]
)

// SweepReport is the node form of the public sweep report.
type SweepReport = sweepReport

// NewReport returns an empty report naming the pair, its paths and request.
func NewReport(a, b *topo.Body, pathA, pathB vocab.PairPath, req vocab.SweepRequest) *SweepReport {
	return &sweepReport{A: a, B: b, PathA: pathA, PathB: pathB, Request: req}
}

// nodeTrack wraps a producer's exact track as the report's track.
func nodeTrack(track *sweepproof.Track) *vocab.SweepContactTrackOf[*topo.Face, *topo.Edge, *topo.Vertex] {
	return vocab.NewSweepContactTrack[*topo.Face, *topo.Edge, *topo.Vertex](seal.Key{}, track, track)
}

// trackOf returns the producer's exact track behind a report's track.
func trackOf(track *vocab.SweepContactTrackOf[*topo.Face, *topo.Edge, *topo.Vertex]) *sweepproof.Track {
	if track == nil {
		return nil
	}
	exact, _ := track.Source(seal.Key{}).(*sweepproof.Track)
	return exact
}

type affinePairPath = sweeppath.AffinePath
