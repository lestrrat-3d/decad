// Package verifyrun runs Verify: it audits each body, runs the asked surveys,
// proves every pair and assembles the report. It works on topology nodes; the
// root package lifts the finished report to its handles.
package verifyrun

import (
	"github.com/lestrrat-3d/decad/internal/boolchain"
	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/verifiedmesh"
	"github.com/lestrrat-3d/decad/internal/verifyoption"
	vocab "github.com/lestrrat-3d/decad/report"
)

// Hooks are the services a verification takes from the document it runs in.
type Hooks struct {
	// Mesher meshes a body for the mesh-intersection pair proof.
	Mesher verifiedmesh.Mesher
	// Draft meshes a body at the lowest verification level for the body
	// check's per-shell winding reading, which needs no boundary proof.
	Draft verifiedmesh.DraftMesher
	// Operand meshes a pair operand the verification cache holds no mesh for,
	// as the public Booleans mesh theirs.
	Operand booleaneval.OperandMesher
	// Alloc allocates the scratch bodies a measurement builds.
	Alloc topo.Alloc
	// History optionally supplies the producer of a Cut result.
	History boolchain.Source
}

// The verification records of the public report package, with topology nodes
// as the body and face references.
type (
	Report              = vocab.ReportOf[*topo.Body, *topo.Face, vocab.JointCell]
	BodyReport          = vocab.BodyReportOf[*topo.Body, *topo.Face, vocab.JointCell]
	Diagnostic          = vocab.DiagnosticOf[*topo.Body, vocab.JointCell]
	DiagnosticPair      = vocab.DiagnosticPairOf[*topo.Body]
	Interference        = vocab.InterferenceOf[*topo.Body]
	Clearance           = vocab.ClearanceOf[*topo.Body]
	WallResult          = vocab.WallResultOf[*topo.Body, vocab.JointCell]
	UndercutResult      = vocab.UndercutResultOf[*topo.Body, *topo.Face, vocab.JointCell]
	ConcaveRadiusResult = vocab.ConcaveRadiusResultOf[*topo.Body, vocab.JointCell]
	ValidityResult      = vocab.ValidityResultOf[*topo.Body, vocab.JointCell]
)

type verifyConfig = verifyoption.Config
