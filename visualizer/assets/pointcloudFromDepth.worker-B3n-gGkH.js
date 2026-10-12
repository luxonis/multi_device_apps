/**
* @license
* Copyright 2019 Google LLC
* SPDX-License-Identifier: Apache-2.0
*/
const e=Symbol(`Comlink.proxy`),t=Symbol(`Comlink.endpoint`),n=Symbol(`Comlink.releaseProxy`),r=Symbol(`Comlink.finalizer`),i=Symbol(`Comlink.thrown`),a=e=>typeof e==`object`&&!!e||typeof e==`function`,o=new Map([[`proxy`,{canHandle:t=>a(t)&&t[e],serialize(e){let{port1:t,port2:n}=new MessageChannel;return c(e,t),[n,[n]]},deserialize(e){return e.start(),d(e)}}],[`throw`,{canHandle:e=>a(e)&&i in e,serialize({value:e}){let t;return t=e instanceof Error?{isError:!0,value:{message:e.message,name:e.name,stack:e.stack}}:{isError:!1,value:e},[t,[]]},deserialize(e){throw e.isError?Object.assign(Error(e.value.message),e.value):e.value}}]]);function s(e,t){for(let n of e)if(t===n||n===`*`||n instanceof RegExp&&n.test(t))return!0;return!1}function c(e,t=globalThis,n=[`*`]){t.addEventListener(`message`,function a(o){if(!o||!o.data)return;if(!s(n,o.origin)){console.warn(`Invalid origin '${o.origin}' for comlink proxy`);return}let{id:l,type:d,path:f}=Object.assign({path:[]},o.data),p=(o.data.argumentList||[]).map(T),m;try{let t=f.slice(0,-1).reduce((e,t)=>e[t],e),n=f.reduce((e,t)=>e[t],e);switch(d){case`GET`:m=n;break;case`SET`:t[f.slice(-1)[0]]=T(o.data.value),m=!0;break;case`APPLY`:m=n.apply(t,p);break;case`CONSTRUCT`:m=C(new n(...p));break;case`ENDPOINT`:{let{port1:t,port2:n}=new MessageChannel;c(e,n),m=S(t,[t])}break;case`RELEASE`:m=void 0;break;default:return}}catch(e){m={value:e,[i]:0}}Promise.resolve(m).catch(e=>({value:e,[i]:0})).then(n=>{let[i,o]=w(n);t.postMessage(Object.assign(Object.assign({},i),{id:l}),o),d===`RELEASE`&&(t.removeEventListener(`message`,a),u(t),r in e&&typeof e[r]==`function`&&e[r]())}).catch(e=>{let[n,r]=w({value:TypeError(`Unserializable return value`),[i]:0});t.postMessage(Object.assign(Object.assign({},n),{id:l}),r)})}),t.start&&t.start()}function l(e){return e.constructor.name===`MessagePort`}function u(e){l(e)&&e.close()}function d(e,t){let n=new Map;return e.addEventListener(`message`,function(e){let{data:t}=e;if(!t||!t.id)return;let r=n.get(t.id);if(r)try{r(t)}finally{n.delete(t.id)}}),v(e,n,[],t)}function f(e){if(e)throw Error(`Proxy has been released and is not useable`)}function p(e){return E(e,new Map,{type:`RELEASE`}).then(()=>{u(e)})}const m=new WeakMap,h=`FinalizationRegistry`in globalThis&&new FinalizationRegistry(e=>{let t=(m.get(e)||0)-1;m.set(e,t),t===0&&p(e)});function g(e,t){let n=(m.get(t)||0)+1;m.set(t,n),h&&h.register(e,t,e)}function _(e){h&&h.unregister(e)}function v(e,r,i=[],a=function(){}){let o=!1,s=new Proxy(a,{get(t,a){if(f(o),a===n)return()=>{_(s),p(e),r.clear(),o=!0};if(a===`then`){if(i.length===0)return{then:()=>s};let t=E(e,r,{type:`GET`,path:i.map(e=>e.toString())}).then(T);return t.then.bind(t)}return v(e,r,[...i,a])},set(t,n,a){f(o);let[s,c]=w(a);return E(e,r,{type:`SET`,path:[...i,n].map(e=>e.toString()),value:s},c).then(T)},apply(n,a,s){f(o);let c=i[i.length-1];if(c===t)return E(e,r,{type:`ENDPOINT`}).then(T);if(c===`bind`)return v(e,r,i.slice(0,-1));let[l,u]=b(s);return E(e,r,{type:`APPLY`,path:i.map(e=>e.toString()),argumentList:l},u).then(T)},construct(t,n){f(o);let[a,s]=b(n);return E(e,r,{type:`CONSTRUCT`,path:i.map(e=>e.toString()),argumentList:a},s).then(T)}});return g(s,e),s}function y(e){return Array.prototype.concat.apply([],e)}function b(e){let t=e.map(w);return[t.map(e=>e[0]),y(t.map(e=>e[1]))]}const x=new WeakMap;function S(e,t){return x.set(e,t),e}function C(t){return Object.assign(t,{[e]:!0})}function w(e){for(let[t,n]of o)if(n.canHandle(e)){let[r,i]=n.serialize(e);return[{type:`HANDLER`,name:t,value:r},i]}return[{type:`RAW`,value:e},x.get(e)||[]]}function T(e){switch(e.type){case`HANDLER`:return o.get(e.name).deserialize(e.value);case`RAW`:return e.value}}function E(e,t,n,r){return new Promise(i=>{let a=D();t.set(a,i),e.start&&e.start(),e.postMessage(Object.assign({id:a},n),r)})}function D(){return[,,,,].fill(0).map(()=>Math.floor(Math.random()*(2**53-1)).toString(16)).join(`-`)}const O=Object.freeze({debugLogs:!1,streamDiagnostics:!1});function k(e){if(e!=null)return Number.isFinite(e)&&e>0?e:8e3}function A(e){return{neuralCameraIntrinsics:e?.neuralCameraIntrinsics,stereoDepthMaxValue:k(e?.stereoDepthMaxValue),perspective:e?.perspective,debugLogs:e?.debugLogs??O.debugLogs,streamDiagnostics:e?.streamDiagnostics??O.streamDiagnostics}}function j(){return A({})}let M=j();const N=new Set;function P(e){return A(e)}function F(){for(let e of N)e(M)}function I(){return M}function L(e){return M=P({...M,...e}),F(),M}var R=class e{static instance=null;constructor(){}static get debugLogsEnabled(){return I().debugLogs??!1}static set debugLogsEnabled(e){L({debugLogs:e})}log(t,...n){e.debugLogsEnabled&&console.log(t,...n)}info(t,...n){e.debugLogsEnabled&&console.log(t,...n)}debug(t,...n){e.debugLogsEnabled&&console.debug(t,...n)}warn(t,...n){e.debugLogsEnabled&&console.warn(t,...n)}error(t,...n){e.debugLogsEnabled&&console.error(t,...n)}static getLogger(){return e.instance||=new e,e.instance}};new Uint8Array(new Uint16Array([1]).buffer)[0];function z(e){let t=e?.fullRange===!1,n=t?255/224:1,r=e?.matrix===`bt709`;return{yOffset:t?16:0,yScale:t?255/219:1,redV:(r?1.5748:1.402)*n,greenU:(r?.187324:.344136)*n,greenV:(r?.468124:.714136)*n,blueU:(r?1.8556:1.772)*n}}function B(e,t,n,r,i,a,o){let{yOffset:s,yScale:c,redV:l,greenU:u,greenV:d,blueU:f}=z(o),p=t*n,m=p,h=p+p/4,g=a.byteLength/16;if(t===r&&n===i&&r>0&&i>0&&r%2==0&&i%2==0&&g===p){let t=12;for(let n=0;n<i;n++){let i=n*r,o=(n>>1)*(r/2);for(let n=0;n<r;n+=2){let r=o+(n>>1),p=(e[m+r]??128)-128,g=(e[h+r]??128)-128,_=l*g,v=u*p,y=d*g,b=f*p;for(let r=0;r<2;r++){let o=((e[i+n+r]??0)-s)*c;a[t]=Math.min(255,Math.max(0,o+_)),a[t+1]=Math.min(255,Math.max(0,o-v-y)),a[t+2]=Math.min(255,Math.max(0,o+b)),a[t+3]=255,t+=16}}}return a}let _=new DataView(a.buffer,a.byteOffset,a.byteLength);for(let a=0;a<g;a++){let o=a%r,p=Math.floor(a/r),g=Math.floor(t/r*o),v=Math.floor(n/i*p);if(g>=t||v>=n)continue;let y=v*t+g,b=Math.floor(v/2)*(t/2)+Math.floor(g/2),x=((e[y]??0)-s)*c,S=(e[m+b]??128)-128,C=(e[h+b]??128)-128,w=x+l*C,T=x-u*S-d*C,E=x+f*S;w=Math.min(255,Math.max(0,w)),T=Math.min(255,Math.max(0,T)),E=Math.min(255,Math.max(0,E));let D=a*16;_.setUint8(D+12,w),_.setUint8(D+13,T),_.setUint8(D+14,E),_.setUint8(D+15,255)}return a}const V=R.getLogger();function H(e){return S(e,e.map(e=>e.buffer))}function U(e,t,n,r,i,a,o,s){let c=t*n,l=new ArrayBuffer(c*16),u=new Float32Array(l),d=new Uint8Array(l);for(let c=0;c<n;c++)for(let n=0;n<t;n++){let l=c*t+n,f=e[l]??0,p=0,m=0,h=0,g=f*1;g>0&&r!==0&&i!==0&&(p=(n-a)*g/r,m=(o-c)*g/i,h=g,h>s&&(p=0,m=0,h=0));let _=l*4;u[_]=p,u[_+1]=m,u[_+2]=h,d[l*16+15]=255}return d}function W(e){e.intrinsics.destroy(),e.output.destroy(),e.readback.destroy()}function G(e,t){let n=e.idleBuffers;if(e.idleBuffers=void 0,n?.outputSize===t)return n;n&&W(n);let r=e.device.createBuffer({label:`Intrinsics Buffer`,size:80,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_DST}),i;try{i=e.device.createBuffer({label:`Output XYZColor Buffer (Combined)`,size:t,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_SRC});let n=e.device.createBuffer({label:`Readback Buffer`,size:t,usage:GPUBufferUsage.COPY_DST|GPUBufferUsage.MAP_READ});return{outputSize:t,intrinsics:r,output:i,readback:n}}catch(e){throw r.destroy(),i?.destroy(),e}}let K,q;async function J(){if(K)return K;if(q)return await q;q=(async()=>{if(navigator.gpu===void 0)return;let e=await navigator.gpu.requestAdapter();if(!e)return;let t=await e.requestDevice(),n=t.createBindGroupLayout({label:`Compute Bind Group Layout`,entries:[{binding:0,visibility:GPUShaderStage.COMPUTE,buffer:{type:`read-only-storage`}},{binding:1,visibility:GPUShaderStage.COMPUTE,buffer:{type:`read-only-storage`}},{binding:2,visibility:GPUShaderStage.COMPUTE,buffer:{type:`storage`}},{binding:3,visibility:GPUShaderStage.COMPUTE,buffer:{type:`read-only-storage`}}]}),r=t.createShaderModule({label:`Compute Shader Module`,code:`
struct Intrinsics {
  fx: f32,
  fy: f32,
  cx: f32,
  cy: f32,
  depthScale: f32,
  depthWidth: f32,
  depthHeight: f32,
  i420Width: f32,
  i420Height: f32,
  numFrames_f: f32,
  bytesPerI420Frame_f: f32,
  maxSteroDepth: f32,
  yOffset: f32,
  yScale: f32,
  redV: f32,
  greenU: f32,
  greenV: f32,
  blueU: f32,
};

struct OutputPointData {
  x: f32,
  y: f32,
  z: f32,
  color: u32,
};

@group(0) @binding(0) var<storage, read> depthBuffer: array<u32>;
@group(0) @binding(1) var<storage, read> intrinsics: Intrinsics;
@group(0) @binding(2) var<storage, read_write> xyzColorBuffer: array<OutputPointData>;
@group(0) @binding(3) var<storage, read> i420Buffer: array<u32>;

fn getI420Byte(byteIndexWithinFrame: u32, frameOffsetElements: u32, bufferLengthElements: u32) -> u32 {
  let globalByteIndex = (frameOffsetElements * 4u) + byteIndexWithinFrame;
  let elementIndex = globalByteIndex / 4u;
  let offsetInElement = globalByteIndex % 4u;
  if (elementIndex >= bufferLengthElements) {
    return 0u;
  }
  let packedValue = i420Buffer[elementIndex];
  return (packedValue >> (offsetInElement * 8u)) & 0xFFu;
}

@compute @workgroup_size(128, 1, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
  let pixel_idx_in_frame = global_id.x;
  let frame_idx = global_id.y;
  let depthWidth = u32(intrinsics.depthWidth);
  let depthHeight = u32(intrinsics.depthHeight);
  let i420WidthU = u32(intrinsics.i420Width);
  let i420HeightU = u32(intrinsics.i420Height);
  let numFrames = u32(intrinsics.numFrames_f);
  let bytesPerI420Frame = u32(intrinsics.bytesPerI420Frame_f);
  let numPixelsPerFrame = depthWidth * depthHeight;
  if (pixel_idx_in_frame >= numPixelsPerFrame || frame_idx >= numFrames || depthWidth == 0u || depthHeight == 0u) {
    return;
  }
  let global_output_idx = (frame_idx * numPixelsPerFrame) + pixel_idx_in_frame;
  let depth_elements_per_frame = (numPixelsPerFrame + 1u) / 2u;
  let depth_frame_offset = frame_idx * depth_elements_per_frame;
  let i420AlignedStrideBytes = (bytesPerI420Frame + 3u) / 4u * 4u;
  let i420_elements_per_frame_aligned = i420AlignedStrideBytes / 4u;
  let i420_frame_offset = frame_idx * i420_elements_per_frame_aligned;
  let depthIndex32_within_frame = pixel_idx_in_frame / 2u;
  let packedDepthU32_idx = depth_frame_offset + depthIndex32_within_frame;
  if (packedDepthU32_idx >= arrayLength(&depthBuffer)) {
    return;
  }
  let packedDepthU32 = depthBuffer[packedDepthU32_idx];
  var rawDepthU16: u32;
  if (pixel_idx_in_frame % 2u == 0u) {
    rawDepthU16 = packedDepthU32 & 0xFFFFu;
  } else {
    rawDepthU16 = (packedDepthU32 >> 16u) & 0xFFFFu;
  }
  let u = f32(pixel_idx_in_frame % depthWidth);
  let v = f32(pixel_idx_in_frame / depthWidth);
  var outputPoint: OutputPointData;
  outputPoint.x = 0.0;
  outputPoint.y = 0.0;
  outputPoint.z = 0.0;
  outputPoint.color = 0xFF000000u;
  let fx = intrinsics.fx;
  let fy = intrinsics.fy;
  let depthScale = intrinsics.depthScale;
  let maxSteroDepth = intrinsics.maxSteroDepth;
  if (rawDepthU16 > 0u && fx != 0.0 && fy != 0.0 && depthScale > 0.0) {
    let z = f32(rawDepthU16) * depthScale;
    let x = (u - intrinsics.cx) * z / fx;
    let y = (intrinsics.cy - v) * z / fy;

    // Filter out points beyond maxSteroDepth
    if (z <= maxSteroDepth) {
      outputPoint.x = x;
      outputPoint.y = y;
      outputPoint.z = z;

      if (i420WidthU > 0u && i420HeightU > 0u && bytesPerI420Frame > 0u) {
        let i420X = min(i420WidthU - 1u, u32(floor(u * (intrinsics.i420Width / intrinsics.depthWidth))));
        let i420Y = min(i420HeightU - 1u, u32(floor(v * (intrinsics.i420Height / intrinsics.depthHeight))));
        let frameSizeY = i420WidthU * i420HeightU;
        let uvWidth = (i420WidthU + 1u) / 2u;
        let uvHeight = (i420HeightU + 1u) / 2u;
        let frameSizeUV = uvWidth * uvHeight;
        let uOffsetBytes = frameSizeY;
        let vOffsetBytes = frameSizeY + frameSizeUV;
        let yIndexBytes = i420Y * i420WidthU + i420X;
        let uvX = i420X / 2u;
        let uvY = i420Y / 2u;
        let uvIndexBytes = uvY * uvWidth + uvX;
        let uIndexBytes = uOffsetBytes + uvIndexBytes;
        let vIndexBytes = vOffsetBytes + uvIndexBytes;
        let i420BufferTotalElements = arrayLength(&i420Buffer);
        if (yIndexBytes < bytesPerI420Frame && uIndexBytes < bytesPerI420Frame && vIndexBytes < bytesPerI420Frame && (i420_frame_offset + (vIndexBytes / 4u)) < i420BufferTotalElements) {
          let y_byte = getI420Byte(yIndexBytes, i420_frame_offset, i420BufferTotalElements);
          let u_byte = getI420Byte(uIndexBytes, i420_frame_offset, i420BufferTotalElements);
          let v_byte = getI420Byte(vIndexBytes, i420_frame_offset, i420BufferTotalElements);
          let y_f = (f32(y_byte) - intrinsics.yOffset) * intrinsics.yScale;
          let u_f = f32(u_byte) - 128.0;
          let v_f = f32(v_byte) - 128.0;
          var r_f = y_f + intrinsics.redV * v_f;
          var g_f = y_f - intrinsics.greenU * u_f - intrinsics.greenV * v_f;
          var b_f = y_f + intrinsics.blueU * u_f;
          let r = u32(clamp(r_f, 0.0, 255.0)) & 0xFFu;
          let g = u32(clamp(g_f, 0.0, 255.0)) & 0xFFu;
          let b = u32(clamp(b_f, 0.0, 255.0)) & 0xFFu;
          let a = 255u;
          outputPoint.color = (r) | (g << 8u) | (b << 16u) | (a << 24u);
        }
      }
    }
  }
  if (global_output_idx < arrayLength(&xyzColorBuffer)) {
    xyzColorBuffer[global_output_idx] = outputPoint;
  }
}
`}),i={device:t,bindGroupLayout:n,pipeline:t.createComputePipeline({label:`Compute Pipeline`,layout:t.createPipelineLayout({bindGroupLayouts:[n]}),compute:{module:r,entryPoint:`main`}})};return t.lost.then(()=>{i.idleBuffers&&=(W(i.idleBuffers),void 0),K?.device===t&&(K=void 0),q=void 0}),K=i,i})();try{return await q}finally{q=void 0}}async function Y(e,t,n,r,i,a,o,s,c,l,u,{hasGPU:d,colorSpace:f}){let p=e.length;if(p===0||p!==s.length)return V.warn(`GPU: Input frame arrays are empty or mismatched.`),[];if(t<=0||n<=0)throw Error(`GPU: Invalid depth dimensions!`);if(e[0]&&!(e[0]instanceof Uint16Array))throw Error(`GPU: depthFrames[0].data is NOT a Uint16Array!`);if(s[0]&&!(s[0]instanceof Uint8Array))throw Error(`GPU: i420Frames[0].data is NOT a Uint8Array!`);if(!d){let d=[];for(let m=0;m<p;m++){let p=e[m],h=s[m];if(!p||!h){V.warn(`CPU Fallback: Skipping frame ${m} due to missing data.`),d.push(new Uint8Array);continue}try{let e=U(p,t,n,r,i,a,o,u);d.push(B(h,c,l,t,n,e,f))}catch(e){V.error(`CPU Fallback: Error processing frame ${m}:`,e),d.push(new Uint8Array)}}return H(d)}let m=null,h,g,_=!1,v=null,y=null,b=null,x=null,S=null;try{if(h=await J(),!h)throw Error(`Failed to get GPU adapter.`);m=h.device;let d=m.queue,C=t*n,w=C*16,T=w*p,E=C*2,D=Math.ceil(E/4),O=D*p*4,k=s[0]?.byteLength??0;if(k===0&&p>0)throw Error(`GPU: I420 frame data is empty or invalid for size calculation.`);let A=Math.ceil(k/4)*4,j=A*p,M=z(f),N=new Float32Array([r,i,a,o,1,t,n,c,l,p,k,u,M.yOffset,M.yScale,M.redV,M.greenU,M.greenV,M.blueU]);g=G(h,T),y=g.intrinsics,x=g.output,S=g.readback,d.writeBuffer(y,0,N.buffer,N.byteOffset,N.byteLength),v=m.createBuffer({label:`Depth Buffer (Combined)`,size:O,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_DST,mappedAtCreation:!0});let P=v.getMappedRange(),F=new Uint32Array(P);for(let t=0;t<p;++t){let n=e[t];if(!n||!(n instanceof Uint16Array)){V.warn(`GPU: Skipping depth frame ${t} due to missing/invalid data.`);let e=t*D,n=D;F.fill(0,e,e+n);continue}let r=t*D;new Uint16Array(P,r*4,C).set(n.subarray(0,C))}v.unmap(),b=m.createBuffer({label:`I420 Buffer (Combined, Aligned)`,size:j,usage:GPUBufferUsage.STORAGE|GPUBufferUsage.COPY_DST,mappedAtCreation:!0});let I=b.getMappedRange(),L=new Uint8Array(I);for(let e=0;e<p;++e){let t=s[e],n=e*A;if(!t||!(t instanceof Uint8Array)||t.byteLength!==k){V.warn(`GPU: Skipping I420 frame ${e} due to missing/invalid data or size mismatch.`),L.fill(0,n,n+A);continue}if(new Uint8Array(I,n,t.byteLength).set(t),A>t.byteLength){let e=n+t.byteLength,r=A-t.byteLength;L.fill(0,e,e+r)}}b.unmap();let R=m.createBindGroup({label:`Compute Bind Group`,layout:h.bindGroupLayout,entries:[{binding:0,resource:{buffer:v}},{binding:1,resource:{buffer:y}},{binding:2,resource:{buffer:x}},{binding:3,resource:{buffer:b}}]}),B=m.createCommandEncoder({label:`Compute Command Encoder`}),U=B.beginComputePass({label:`Compute Pass`});U.setPipeline(h.pipeline),U.setBindGroup(0,R);let W=Math.ceil(C/128),K=p;U.dispatchWorkgroups(W,K,1),U.end(),B.copyBufferToBuffer(x,0,S,0,T),d.submit([B.finish()]),await S.mapAsync(GPUMapMode.READ);let q=S.getMappedRange(),Y=new Uint8Array(q.slice(0));if(S.unmap(),_=!0,p===1)return H([Y]);let X=[];for(let e=0;e<p;e++){let t=e*w,n=Math.min(t+w,Y.byteLength),r=Y.slice(t,n);r.byteLength===w?X.push(r):(V.warn(`GPU: Result frame ${e} has incorrect size (${r.byteLength} vs ${w}). Padding or error occurred.`),X.push(new Uint8Array(w)))}return H(X)}catch(d){_=!1,V.error(`GPU: Error during WebGPU processing:`,d),V.warn(`GPU: Falling back to CPU due to error.`);let m=[];for(let d=0;d<p;d++){let p=e[d],h=s[d];if(!p||!h){V.warn(`CPU Fallback (error path): Skipping frame ${d} due to missing data.`),m.push(new Uint8Array);continue}try{let e=U(p,t,n,r,i,a,o,u);m.push(B(h,c,l,t,n,e,f))}catch(e){V.error(`CPU Fallback (error path): Error processing frame ${d}:`,e),m.push(new Uint8Array)}}return H(m)}finally{v?.destroy(),b?.destroy(),g&&(g.readback.mapState===`mapped`&&g.readback.unmap(),_&&h&&h===K&&!h.idleBuffers?h.idleBuffers=g:W(g))}}async function X(e,t,n,r,i,a,o,s,c,l,u){return await Y(e,t,n,r,i,a,o,s,c,l,u,{hasGPU:!1})}c({depthToPointcloud:X,depthToPointcloudGPU:Y});