use anyhow::Error;

use crate::prelude::*;

use super::{
    viewport_3d::{EdgeDrawMode, FaceDrawMode, Viewport3dSettings},
    viewport_split::SplitTree,
};

pub struct ApplicationContext {
    /// The mesh is at the center of the application
    /// - The graph generates a program that produces this mesh.
    /// - The 3d viewport renders this mesh.
    pub mesh: Option<HalfEdgeMesh>,
    /// The tree of splits at the center of application. Splits recursively
    /// partition the state either horizontally or vertically. This separation
    /// is dynamic, very similar to Blender's UI model
    pub split_tree: SplitTree,
}

impl ApplicationContext {
    pub fn new() -> ApplicationContext {
        ApplicationContext {
            mesh: None,
            split_tree: SplitTree::default_tree(),
        }
    }

    pub fn setup(&self, render_ctx: &mut RenderContext) {
        render_ctx.add_light(r3::DirectionalLight {
            color: glam::Vec3::ONE,
            intensity: 10.0,
            // Direction will be normalized
            direction: glam::Vec3::new(-1.0, -4.0, 2.0),
            distance: 400.0,
        });
    }

    pub fn update(
        &mut self,
        egui_ctx: &egui::CtxRef,
        editor_state: &mut graph::GraphEditorState,
        render_ctx: &mut RenderContext,
        viewport_settings: &Viewport3dSettings,
    ) {
        // TODO: Instead of clearing all objects, make the app context own the
        // objects it's drawing and clear those instead.
        render_ctx.clear_objects();

        if let Err(err) = self.compile_and_update_mesh(editor_state) {
            self.paint_errors(egui_ctx, err);
        }
        if let Err(err) = self.run_side_effects(editor_state) {
            eprintln!("There was an errror executing side effect: {}", err);
        }
        if let Err(err) = self.build_and_render_mesh(render_ctx, viewport_settings) {
            self.paint_errors(egui_ctx, err);
        }
    }

    pub fn build_and_render_mesh(
        &mut self,
        render_ctx: &mut RenderContext,
        viewport_settings: &Viewport3dSettings,
    ) -> Result<()> {
        if let Some(mesh) = self.mesh.as_ref() {
            // Base mesh
            {
                if let Some(VertexIndexBuffers {
                    positions,
                    normals,
                    indices,
                }) = match viewport_settings.face_mode {
                    FaceDrawMode::Flat => Some(mesh.generate_triangle_buffers_flat()),
                    FaceDrawMode::Smooth => Some(mesh.generate_triangle_buffers_smooth()?),
                    FaceDrawMode::None => None,
                } {
                    if !positions.is_empty() {
                        render_ctx.face_routine.add_base_mesh(
                            &render_ctx.renderer,
                            &positions,
                            &normals,
                            &indices,
                        );
                    }
                }
            }

            // Face overlays
            {
                let FaceOverlayBuffers { positions, colors } = mesh.generate_face_overlay_buffers();
                if !positions.is_empty() {
                    render_ctx.face_routine.add_overlay_mesh(
                        &render_ctx.renderer,
                        &positions,
                        &colors,
                    );
                }
            }

            // Edges
            {
                if let Some(LineBuffers { positions, colors }) = match viewport_settings.edge_mode {
                    EdgeDrawMode::HalfEdge => Some(mesh.generate_halfedge_arrow_buffers()?),
                    EdgeDrawMode::FullEdge => Some(mesh.generate_line_buffers()?),
                    EdgeDrawMode::None => None,
                } {
                    if !positions.is_empty() {
                        render_ctx.wireframe_routine.add_wireframe(
                            &render_ctx.renderer.device,
                            &positions,
                            &colors,
                        )
                    }
                }
            }

            // Vertices
            {
                let PointBuffers { positions } = mesh.generate_point_buffers();
                if !positions.is_empty() {
                    render_ctx
                        .point_cloud_routine
                        .add_point_cloud(&render_ctx.renderer.device, &positions);
                }
            }
        }
        Ok(())
    }

    pub fn paint_errors(&mut self, egui_ctx: &egui::CtxRef, err: Error) {
        let painter = egui_ctx.debug_painter();
        let width = egui_ctx.available_rect().width();
        painter.text(
            egui::pos2(width - 10.0, 30.0),
            egui::Align2::RIGHT_TOP,
            format!("{}", err),
            egui::TextStyle::Body,
            egui::Color32::RED,
        );
    }

    pub fn compile_and_update_mesh(
        &mut self,
        editor_state: &graph::GraphEditorState,
    ) -> Result<()> {
        if let Some(active) = editor_state.user_state.active_node {
            let program = crate::graph::graph_compiler::compile_graph(&editor_state.graph, active)?;
            let mesh = program.execute()?;
            self.mesh = Some(mesh);
        } else {
            self.mesh = None
        }
        Ok(())
    }

    pub fn run_side_effects(&mut self, editor_state: &mut graph::GraphEditorState) -> Result<()> {
        if let Some(side_effect) = editor_state.user_state.run_side_effect.take() {
            let program =
                crate::graph::graph_compiler::compile_graph(&editor_state.graph, side_effect)?;
            // We ignore the result. The program is only executed to produce a
            // side effect (e.g. exporting a mesh as OBJ)
            let _ = program.execute();
        }
        Ok(())
    }
}

impl Default for ApplicationContext {
    fn default() -> Self {
        Self::new()
    }
}