var gui = {}
gui.messages = null;

$(document).ready(function(){
  get_lang('messages', init_interface)
});

function init_interface(lang_file){
  
  
  console.log(lang_file)
    $.ajax({
      url: lang_file,
      dataType: 'json', 
      success: function(data){
        $('div>h1').html(data.messages.main);
        gui.messages = data.messages;
      },
      error: function(){
        console.log('error loading file');
      },
      async: false
    });
  $('#start_quest').html(gui.messages.start)


  $('.btn-index-game').click(function(){
    // // load questionnaire page
    window.location = 'questions.html';
  });
};

