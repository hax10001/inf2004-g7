'use strict';
const {test}=require('node:test'),assert=require('node:assert/strict');
const {csvField}=require('../server/recording_export');
test('CSV escaping prevents spreadsheet formula execution and line breaking',()=>{
 assert.equal(csvField('=CMD()'),"'=CMD()");assert.equal(csvField('hello,world'),'"hello,world"');
 assert.equal(csvField('quote"me'),'"quote""me"');assert.equal(csvField('normal'),'normal');
});
