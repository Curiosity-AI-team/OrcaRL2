# OrcaRL2

https://json-generator.com/

[
  '{{repeat(5, 7)}}',
  {
    _id: '{{objectId()}}',
    registered: '{{date(new Date(2014, 0, 1), new Date(), "YYYY-MM-ddThh:mm:ss Z")}}',
    picture: 'http://placehold.it/32x32',
    address: '{{integer(100, 999)}} {{street()}}, {{city()}}, {{state()}}, {{integer(100, 10000)}}',
    latitude: '{{floating(-90.000001, 90)}}',
    longitude: '{{floating(-180.000001, 180)}}',
    altitude: '{{floating(-180.000001, 180)}}',
    map: '{{firstName()}}',
    map_frame: 
    [{ 
      x: '0', 
      y: '0', 
      z: '0',
      rx: '0',       
      ry: '0',
      rz: '0',
      rw: '1'
    }],
    robot: '{{firstName()}}',
    robot_frame: 
    [{ 
      x: '{{floating(-90.000001, 90)}}', 
      y: '{{floating(-90.000001, 90)}}', 
      z: '{{floating(-90.000001, 90)}}',
      rx: '{{floating(-90.000001, 90)}}',       
      ry: '{{floating(-90.000001, 90)}}',
      rz: '{{floating(-90.000001, 90)}}',
      rw: '{{floating(-90.000001, 90)}}'
    }],
    target: '{{firstName()}}',
    target_frame: 
    [{ 
      x: '{{floating(-90.000001, 90)}}', 
      y: '{{floating(-90.000001, 90)}}', 
      z: '{{floating(-90.000001, 90)}}',
      rx: '{{floating(-90.000001, 90)}}',       
      ry: '{{floating(-90.000001, 90)}}',
      rz: '{{floating(-90.000001, 90)}}',
      rw: '{{floating(-90.000001, 90)}}'
    }],
    emotion_name: '{{firstName()}}',
    emotion_1: '{{floating(-90.000001, 90)}}',
    emotion_2: '{{floating(-90.000001, 90)}}',
    reserved1: '{{firstName()}}',
    reserved2: '{{floating(-90.000001, 90)}}',
    reserved3: '{{floating(-90.000001, 90)}}'
  }
]