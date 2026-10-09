// Read-only browser regression checks, against an isolated --preview server.
// Optional CONSOLE_BROWSER_EXECUTABLE / CONSOLE_BROWSER_ARGS select a real GPU.
const { chromium } = require('playwright');
const assert = require('node:assert/strict');

(async () => {
  const browser = await chromium.launch({
    headless: true, chromiumSandbox: true,
    executablePath: process.env.CONSOLE_BROWSER_EXECUTABLE || undefined,
    args: JSON.parse(process.env.CONSOLE_BROWSER_ARGS || '["--enable-gpu"]'),
  });
  try {
    for (const mode of ['normal', 'no-antialias', 'no-webgl']) {
      const page = await browser.newPage({ viewport: { width: 1440, height: 1050 } });
      const errors = [];
      page.on('pageerror', error => errors.push(error.message));
      await page.addInitScript(mode => {
        const original = HTMLCanvasElement.prototype.getContext;
        window.webglAttempts = [];
        HTMLCanvasElement.prototype.getContext = function(type, attributes) {
          if (type === 'webgl2') {
            window.webglAttempts.push(attributes);
            if (mode === 'no-webgl' || (mode === 'no-antialias' && attributes?.antialias)) return null;
          }
          return original.call(this, type, attributes);
        };
      }, mode);
      await page.goto(process.env.CONSOLE_URL || 'http://127.0.0.1:8765');
      assert(await page.evaluate(async () => (await (await fetch('/api/bootstrap')).json()).preview), 'Use --preview only');
      await page.waitForFunction(() => document.querySelector('#joint-inputs input'));
      await page.waitForFunction(() => document.querySelector('#joint-count').textContent.includes('14')
        && document.querySelector('#send-joints').disabled);
      if (mode === 'no-webgl') {
        assert.match(await page.locator('#model-loading').innerText(), /WebGL 不可用.*WebGL2.*图形加速/);
        assert.equal(await page.locator('#viewport canvas').count(), 0);
        assert.equal(await page.locator('#send-joints').isDisabled(), true);
        assert.equal(await page.evaluate(() => window.webglAttempts.length), 2);
      } else {
        await page.waitForFunction(() => window.consoleMeshCount > 10 && window.consoleScene.frames > 2, { timeout: 60000 });
        const result = await page.evaluate(() => {
          const view = window.consoleScene;
          const gl = view.renderer.getContext();
          const ext = gl.getExtension('WEBGL_debug_renderer_info');
          view.renderer.render(view.scene, view.camera);
          const canvas = document.createElement('canvas'); canvas.width = canvas.height = 100;
          const ctx = canvas.getContext('2d'); ctx.drawImage(view.renderer.domElement, 0, 0, 100, 100);
          const data = ctx.getImageData(0, 0, 100, 100).data;
          const colors = new Set(); let dark = 0;
          for (let i = 0; i < data.length; i += 4) {
            colors.add(`${data[i]},${data[i+1]},${data[i+2]}`);
            if (data[i] < 180) dark++;
          }
          return { antialias: gl.getContextAttributes().antialias,
            renderer: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : '',
            attempts: window.webglAttempts.length, colors: colors.size, dark,
            triangles: view.renderer.info.render.triangles, meshes: window.consoleMeshCount };
        });
        assert(result.triangles > 500000 && result.colors > 150 && result.dark > 50, JSON.stringify(result));
        assert.equal(result.attempts, mode === 'normal' ? 1 : 2);
        if (mode === 'no-antialias') assert.equal(result.antialias, false);
        if (process.env.CONSOLE_EXPECT_RENDERER) assert(result.renderer.includes(process.env.CONSOLE_EXPECT_RENDERER), result.renderer);
        if (mode === 'normal') await page.screenshot({ path: '/tmp/ieir-web-webgl-fixed.png', fullPage: true });
        console.log(mode, JSON.stringify(result));
      }
      assert.deepEqual(errors, []);
      await page.close();
    }
    console.log('PASS: WebGL2 rendering, antialias fallback, blocked context with usable console');
  } finally { await browser.close(); }
})().catch(error => { console.error(error); process.exitCode = 1; });
