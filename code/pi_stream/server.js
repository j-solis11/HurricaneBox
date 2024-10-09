const express = require('express');
const app = express();
const path = require('path');

//app.use('/hls', express.static(path.join(__dirname, 'public')));
app.use(express.static('public'));
app.use('/hls', express.static('../hls'));

const PORT = 8080;
app.listen(PORT, () => {
	console.log('Server is running on http://192.168.1.37:8080');
});