// Package motionengine is the motion check's engine: it moves one or more
// groups of bodies along a path, evaluates every mover-and-partner pair at
// bisected poses, certifies the intervals between them and assembles the
// report. VerifyMotion, VerifyLinkage and VerifyJointBox drive it; it works on
// topology nodes, and the root package lifts its reports to handles.
package motionengine

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/linkagebound"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/motionoption"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	vocab "github.com/lestrrat-3d/decad/report"
	"github.com/lestrrat-3d/r3"
)

// Hooks are the services a motion check takes from the document it runs in.
type Hooks struct {
	// Alloc allocates the scratch bodies a measurement builds.
	Alloc topo.Alloc
	// Operand meshes a pair operand as the public Booleans mesh theirs.
	Operand booleaneval.OperandMesher
	// Place re-evaluates a payload under a composed motion in the transient
	// document, never committing.
	Place func(ctx context.Context, p topo.Payload, doc *topo.Document, ref provenance.Producer,
		composed r3.Transform) (*topo.Body, error)
	// NewDocument returns the transient document node a check mints pose
	// bodies in: a copy of doc's identity counters over the live bodies, born
	// the way the root package's documents are.
	NewDocument func(doc *topo.Document, live []*topo.Body) *topo.Document
}

// The motion records of the public report package, with topology nodes as the
// body reference.
type (
	Diagnostic     = vocab.DiagnosticOf[*topo.Body, vocab.JointCell]
	DiagnosticPair = vocab.DiagnosticPairOf[*topo.Body]
	Interference   = vocab.InterferenceOf[*topo.Body]
	Clearance      = vocab.ClearanceOf[*topo.Body]
	ValidityResult = vocab.ValidityResultOf[*topo.Body, vocab.JointCell]
	Collision      = vocab.CollisionOf[*topo.Body]
	PoseResult     = vocab.PoseResultOf[*topo.Body]
	// Report is the motion report over nodes; its Motion is left to the caller.
	Report = vocab.MotionReportOf[*topo.Body, any]
)

// Config is the motion check's resolved options.
type Config = motionoption.Config

type (
	cornerBounds  = linkagebound.Bounds
	cornerReading = linkagebound.Reading
)
