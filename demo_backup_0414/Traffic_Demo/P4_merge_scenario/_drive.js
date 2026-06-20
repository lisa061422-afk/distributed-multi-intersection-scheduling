const puppeteer = require('puppeteer-core');
const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

const CHROME = 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe';
const FILE = 'file:///' + path.resolve('merge_scheduling_demo.html').split(path.sep).join('/');
const FPS = 60, ENDHOLD = 0.6;

(async () => {
  const browser = await puppeteer.launch({
    executablePath: CHROME, headless: 'new',
    args: ['--no-sandbox', '--disable-gpu', '--hide-scrollbars', '--force-color-profile=srgb']
  });
  const page = await browser.newPage();
  await page.setViewport({ width: 1152, height: 720, deviceScaleFactor: 3 }); // -> 3456x2160
  await page.goto(FILE, { waitUntil: 'networkidle0' });
  await page.evaluate(() => Promise.all([...document.images].map(i => i.complete ? 0 : new Promise(r => { i.onload = i.onerror = r; }))));
  await new Promise(r => setTimeout(r, 200));

  for (const sched of ['optimal', 'fcfs']) {
    const dur = await page.evaluate(s => window.__dur(s), sched);
    const total = dur + ENDHOLD, frames = Math.ceil(total * FPS);
    const fdir = path.resolve('_frames_' + sched);
    fs.rmSync(fdir, { recursive: true, force: true });
    fs.mkdirSync(fdir);
    console.log(`[${sched}] dur=${dur.toFixed(2)}s frames=${frames}`);
    for (let f = 0; f < frames; f++) {
      const t = Math.min(f / FPS, dur);
      await page.evaluate(([s, tt]) => window.__seek(s, tt), [sched, t]);
      await page.screenshot({ path: path.join(fdir, 'f' + String(f).padStart(5, '0') + '.png') });
      if (f % 60 === 0) process.stdout.write(`  ${sched} ${f}/${frames}\r`);
    }
    const out = path.resolve('clip_' + sched + '.mp4');
    execSync(`ffmpeg -y -framerate ${FPS} -i "${path.join(fdir, 'f%05d.png')}" -c:v libx264 -pix_fmt yuv420p -crf 18 -movflags +faststart "${out}"`, { stdio: 'ignore' });
    fs.rmSync(fdir, { recursive: true, force: true });
    console.log(`\n[${sched}] -> ${out}`);
  }
  await browser.close();
  console.log('DONE');
})().catch(e => { console.error(e); process.exit(1); });
