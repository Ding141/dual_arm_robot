import * as THREE from 'three';

// Three r164 requires WebGL2. Retry less demanding attributes on a fresh
// canvas; a failed context must not prevent the rest of the console loading.
export function createRobotRenderer() {
  const failures = [];
  for (const antialias of [true, false]) {
    const canvas = document.createElement('canvas');
    let status = '';
    canvas.addEventListener('webglcontextcreationerror', event => { status = event.statusMessage; });
    const attributes = {
      alpha: true, depth: true, stencil: false, antialias,
      premultipliedAlpha: true, preserveDrawingBuffer: false,
      powerPreference: 'default', failIfMajorPerformanceCaveat: false,
    };
    const context = canvas.getContext('webgl2', attributes);
    if (context) {
      const renderer = new THREE.WebGLRenderer({ canvas, context, ...attributes });
      renderer.domElement.dataset.antialias = String(context.getContextAttributes().antialias);
      return renderer;
    }
    failures.push({ antialias, status: status || '浏览器未提供 WebGL2 上下文' });
  }
  console.error('Robot WebGL2 context creation failed', failures);
  const error = new Error('浏览器无法创建 WebGL2 上下文。请检查 Chrome 图形加速；双显卡电脑可使用独显浏览器入口，详见网页控制台手册。');
  error.name = 'WebGLContextError';
  throw error;
}
