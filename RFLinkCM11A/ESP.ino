

boolean ESP_TX(char *string) {
  boolean success=false;
  
  //10;ESP;Commandes;
  //0123456789012345678
  // 1ère étape on remplace tous les ";" par des "\0" pour pouvoir traiter facilement chaque bloc de commande RFLink
  
  for(byte cpt = 0; cpt < INPUT_COMMAND_SIZE ; cpt++)
    if (string[cpt] == ';')  
      string[cpt] = 0 ;

  // Une fois là, la chaîne contient la commande complète. Testez d'abord si la commande spécifiée correspond à X10
  if (strncasecmp(string+3,"ESP",3) == 0) {     // ESP Command eg. 
    string+=7;									// Pointeur sur la commande ESP
	if (strncasecmp(string,"YEAR=",5) == 0) {   // Set YEAR Command 
		//10;ESP;YEAR=xxxx;
		unsigned int year = atoi( string + 5 );
		if(CM11A.SetYear(year))
			success=true;
	}
  }
  return success;
}
