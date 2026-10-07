// Package sectionaudit checks the area, separation, and nesting of rewritten
// section boundaries before a body is built.
package sectionaudit

import (
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
)

type (
	CurveSegment = sectionrecord.CurveSegment
	LoopRecord   = sectionrecord.LoopRecord
	Point2       = sectionrecord.Point2
	LineSeg      = sectionrecord.LineSeg
	CircleSeg    = sectionrecord.CircleSeg
	ArcSeg       = sectionrecord.ArcSeg
)

var (
	ErrUnitKind    = decaderr.ErrUnitKind
	ErrNotFinite   = decaderr.ErrNotFinite
	ErrDegenerate  = decaderr.ErrDegenerate
	ErrUnsupported = decaderr.ErrUnsupported
)

// Tolerance absorbs floating-point noise in the section rewrite.
const Tolerance = 1e-9

// Entry identifies one boundary walk and its position in a loop.
type Entry struct {
	loop int
	idx  int
	n    int
	w    survey2d.SegmentWalk
}

// NewEntry records one walk and its loop position.
func NewEntry(loop, idx, n int, walk survey2d.SegmentWalk) Entry {
	return Entry{loop: loop, idx: idx, n: n, w: walk}
}

// DiagnosticError preserves the shared audit error and its coordinate detail.
type DiagnosticError struct {
	legacy   error
	detailed string
}

func (e *DiagnosticError) Error() string    { return e.legacy.Error() }
func (e *DiagnosticError) Unwrap() error    { return e.legacy }
func (e *DiagnosticError) Detailed() string { return e.detailed }

// NewError keeps the legacy error while attaching a detailed rendering.
func NewError(legacy error, detailed string) error {
	return &DiagnosticError{legacy: legacy, detailed: detailed}
}

func auditError(legacy error, detailed string) error { return NewError(legacy, detailed) }
