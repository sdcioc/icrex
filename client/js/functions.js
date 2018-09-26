function exportCVSfile(datain, filename){
 
    let csvContent = "data:txt/json;charset=utf-8,";

    csvContent += datain;

    let encodedUri = encodeURI(csvContent);
    let link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", filename);
    document.body.appendChild(link); // Required for FF
 
    link.click();
    link.remove();
}
